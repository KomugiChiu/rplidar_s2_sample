package main

import (
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"strings"
	"sync"
)

// Append channels keep a backlog and deliver every item; all other channels
// are state: a client only ever receives the latest value (coalesced), so a
// slow phone on bad Wi-Fi never falls behind.
var appendChannels = map[string]int{"log": 1500, "events": 300}

// demandChannel reports channels the gateway only produces while watched.
func demandChannel(ch string) bool {
	switch ch {
	case "map", "scan", "plan", "people", "costmap_local", "costmap_global", "detections":
		return true
	}
	return strings.HasPrefix(ch, "inspect:")
}

type Client struct {
	id      string
	addr    string
	subs    map[string]bool
	pending map[string]json.RawMessage
	queued  map[string][]json.RawMessage
	notify  chan struct{}
}

type Hub struct {
	mu      sync.Mutex
	latest  map[string]json.RawMessage
	rings   map[string][]json.RawMessage
	clients map[string]*Client
	extra   map[string]float64 // demand from camera viewers
	demand  map[string]float64
	// OnDemand is called (outside the lock) when the demanded set changes.
	OnDemand func()
}

func NewHub() *Hub {
	return &Hub{
		latest:  map[string]json.RawMessage{},
		rings:   map[string][]json.RawMessage{},
		clients: map[string]*Client{},
		extra:   map[string]float64{},
		demand:  map[string]float64{},
	}
}

func newID() string {
	b := make([]byte, 8)
	_, _ = rand.Read(b)
	return hex.EncodeToString(b)
}

func (h *Hub) AddClient(addr string, chans []string) *Client {
	c := &Client{
		id: newID(), addr: addr, subs: map[string]bool{},
		pending: map[string]json.RawMessage{}, queued: map[string][]json.RawMessage{},
		notify: make(chan struct{}, 1),
	}
	h.mu.Lock()
	h.clients[c.id] = c
	h.mu.Unlock()
	h.Subscribe(c.id, chans)
	return c
}

func (h *Hub) RemoveClient(c *Client) {
	h.mu.Lock()
	delete(h.clients, c.id)
	changed := h.recomputeLocked()
	h.mu.Unlock()
	h.fire(changed)
}

func (h *Hub) ClientCount() int {
	h.mu.Lock()
	defer h.mu.Unlock()
	return len(h.clients)
}

// Subscribe replaces the client's channel set; new channels get a snapshot.
func (h *Hub) Subscribe(id string, chans []string) bool {
	h.mu.Lock()
	c := h.clients[id]
	if c == nil {
		h.mu.Unlock()
		return false
	}
	next := map[string]bool{}
	for _, ch := range chans {
		if ch != "" {
			next[ch] = true
		}
	}
	for ch := range next {
		if c.subs[ch] {
			continue
		}
		if _, ok := appendChannels[ch]; ok {
			backlog := h.rings[ch]
			if len(backlog) > 600 {
				backlog = backlog[len(backlog)-600:]
			}
			c.queued[ch] = append([]json.RawMessage(nil), backlog...)
		} else if v, ok := h.latest[ch]; ok {
			c.pending[ch] = v
		}
	}
	for ch := range c.subs {
		if !next[ch] {
			delete(c.pending, ch)
			delete(c.queued, ch)
		}
	}
	c.subs = next
	changed := h.recomputeLocked()
	h.signal(c)
	h.mu.Unlock()
	h.fire(changed)
	return true
}

func (h *Hub) Set(ch string, data json.RawMessage) {
	h.mu.Lock()
	h.latest[ch] = data
	for _, c := range h.clients {
		if c.subs[ch] {
			c.pending[ch] = data
			h.signal(c)
		}
	}
	h.mu.Unlock()
}

func (h *Hub) Latest(ch string) json.RawMessage {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.latest[ch]
}

func (h *Hub) Append(ch string, items []json.RawMessage) {
	if len(items) == 0 {
		return
	}
	max := appendChannels[ch]
	h.mu.Lock()
	r := append(h.rings[ch], items...)
	if len(r) > 2*max {
		r = append([]json.RawMessage(nil), r[len(r)-max:]...)
	}
	h.rings[ch] = r
	for _, c := range h.clients {
		if c.subs[ch] {
			q := append(c.queued[ch], items...)
			if len(q) > 2000 {
				q = q[len(q)-2000:]
			}
			c.queued[ch] = q
			h.signal(c)
		}
	}
	h.mu.Unlock()
}

// Drain hands the writer everything queued for this client.
func (h *Hub) Drain(c *Client) (map[string]json.RawMessage, map[string][]json.RawMessage) {
	h.mu.Lock()
	defer h.mu.Unlock()
	p, q := c.pending, c.queued
	c.pending = map[string]json.RawMessage{}
	c.queued = map[string][]json.RawMessage{}
	return p, q
}

func (h *Hub) SetExtraDemand(ch string, v float64) {
	h.mu.Lock()
	if v <= 0 {
		delete(h.extra, ch)
	} else {
		h.extra[ch] = v
	}
	changed := h.recomputeLocked()
	h.mu.Unlock()
	h.fire(changed)
}

func (h *Hub) Demand() map[string]float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	out := make(map[string]float64, len(h.demand))
	for k, v := range h.demand {
		out[k] = v
	}
	return out
}

func (h *Hub) signal(c *Client) {
	select {
	case c.notify <- struct{}{}:
	default:
	}
}

func (h *Hub) recomputeLocked() bool {
	d := map[string]float64{}
	for _, c := range h.clients {
		for ch := range c.subs {
			if demandChannel(ch) {
				d[ch] = 1
			}
		}
	}
	for ch, v := range h.extra {
		if v > d[ch] {
			d[ch] = v
		}
	}
	if len(d) == len(h.demand) {
		same := true
		for k, v := range d {
			if h.demand[k] != v {
				same = false
				break
			}
		}
		if same {
			return false
		}
	}
	h.demand = d
	return true
}

func (h *Hub) fire(changed bool) {
	if changed && h.OnDemand != nil {
		h.OnDemand()
	}
}
