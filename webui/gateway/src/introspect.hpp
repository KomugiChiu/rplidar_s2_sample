// Generic ROS message -> JSON using rosidl_typesupport_introspection_cpp.
// Used by the topic inspector and for message types the gateway is not
// compiled against (ai_human_detection/PersonInfoArray).
#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rmw/rmw.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace ti = rosidl_typesupport_introspection_cpp;

class TypeCodec
{
public:
  explicit TypeCodec(const std::string & type)
  {
    lib_ = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    ts_ = rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_cpp", *lib_);
    ilib_ = rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
    auto its = rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_introspection_cpp", *ilib_);
    members_ = static_cast<const ti::MessageMembers *>(its->data);
  }

  // max_arr: arrays longer than this are cut (with an "… (N items)" marker).
  nlohmann::json decode(const rclcpp::SerializedMessage & ser, size_t max_arr) const
  {
    // operator new storage is suitably aligned for any message struct
    std::unique_ptr<uint8_t[]> buf(new uint8_t[members_->size_of_]);
    members_->init_function(buf.get(), rosidl_runtime_cpp::MessageInitialization::ALL);
    auto ret = rmw_deserialize(&ser.get_rcl_serialized_message(), ts_, buf.get());
    nlohmann::json out;
    if (ret == RMW_RET_OK) {
      out = walk(members_, buf.get(), max_arr);
    }
    members_->fini_function(buf.get());
    if (ret != RMW_RET_OK) {
      rmw_reset_error();
      throw std::runtime_error("deserialization failed");
    }
    return out;
  }

private:
  static nlohmann::json walk(const ti::MessageMembers * mm, const void * msg, size_t max_arr)
  {
    nlohmann::json o = nlohmann::json::object();
    for (uint32_t i = 0; i < mm->member_count_; ++i) {
      const ti::MessageMember & m = mm->members_[i];
      const void * field = static_cast<const uint8_t *>(msg) + m.offset_;
      if (!m.is_array_) {
        o[m.name_] = scalar(m, field, max_arr);
        continue;
      }
      size_t n = m.size_function ? m.size_function(field) : m.array_size_;
      size_t lim = std::min(n, max_arr);
      nlohmann::json arr = nlohmann::json::array();
      for (size_t k = 0; k < lim; ++k) {
        if (m.type_id_ == ti::ROS_TYPE_MESSAGE) {
          arr.push_back(walk(static_cast<const ti::MessageMembers *>(m.members_->data),
                             m.get_const_function(field, k), max_arr));
        } else {
          arr.push_back(element(m, field, k));
        }
      }
      if (n > lim) {
        arr.push_back("… (" + std::to_string(n) + " items)");
      }
      o[m.name_] = std::move(arr);
    }
    return o;
  }

  template<typename T>
  static T fetch(const ti::MessageMember & m, const void * field, size_t k)
  {
    T v{};
    if (m.fetch_function) {
      m.fetch_function(field, k, &v);
    } else if (m.get_const_function) {
      v = *static_cast<const T *>(m.get_const_function(field, k));
    }
    return v;
  }

  static nlohmann::json element(const ti::MessageMember & m, const void * field, size_t k)
  {
    switch (m.type_id_) {
      case ti::ROS_TYPE_FLOAT: return fetch<float>(m, field, k);
      case ti::ROS_TYPE_DOUBLE: return fetch<double>(m, field, k);
      case ti::ROS_TYPE_LONG_DOUBLE: return static_cast<double>(fetch<long double>(m, field, k));
      case ti::ROS_TYPE_BOOLEAN: return fetch<bool>(m, field, k);
      case ti::ROS_TYPE_CHAR:
      case ti::ROS_TYPE_OCTET:
      case ti::ROS_TYPE_UINT8: return fetch<uint8_t>(m, field, k);
      case ti::ROS_TYPE_INT8: return fetch<int8_t>(m, field, k);
      case ti::ROS_TYPE_UINT16: return fetch<uint16_t>(m, field, k);
      case ti::ROS_TYPE_INT16: return fetch<int16_t>(m, field, k);
      case ti::ROS_TYPE_UINT32: return fetch<uint32_t>(m, field, k);
      case ti::ROS_TYPE_INT32: return fetch<int32_t>(m, field, k);
      case ti::ROS_TYPE_UINT64: return fetch<uint64_t>(m, field, k);
      case ti::ROS_TYPE_INT64: return fetch<int64_t>(m, field, k);
      case ti::ROS_TYPE_STRING: return fetch<std::string>(m, field, k);
      default: return "<unsupported>";
    }
  }

  static nlohmann::json scalar(const ti::MessageMember & m, const void * f, size_t max_arr)
  {
    switch (m.type_id_) {
      case ti::ROS_TYPE_FLOAT: return *static_cast<const float *>(f);
      case ti::ROS_TYPE_DOUBLE: return *static_cast<const double *>(f);
      case ti::ROS_TYPE_LONG_DOUBLE: return static_cast<double>(*static_cast<const long double *>(f));
      case ti::ROS_TYPE_BOOLEAN: return *static_cast<const bool *>(f);
      case ti::ROS_TYPE_CHAR:
      case ti::ROS_TYPE_OCTET:
      case ti::ROS_TYPE_UINT8: return *static_cast<const uint8_t *>(f);
      case ti::ROS_TYPE_INT8: return *static_cast<const int8_t *>(f);
      case ti::ROS_TYPE_UINT16: return *static_cast<const uint16_t *>(f);
      case ti::ROS_TYPE_INT16: return *static_cast<const int16_t *>(f);
      case ti::ROS_TYPE_UINT32: return *static_cast<const uint32_t *>(f);
      case ti::ROS_TYPE_INT32: return *static_cast<const int32_t *>(f);
      case ti::ROS_TYPE_UINT64: return *static_cast<const uint64_t *>(f);
      case ti::ROS_TYPE_INT64: return *static_cast<const int64_t *>(f);
      case ti::ROS_TYPE_STRING: return *static_cast<const std::string *>(f);
      case ti::ROS_TYPE_MESSAGE:
        return walk(static_cast<const ti::MessageMembers *>(m.members_->data), f, max_arr);
      default: return "<unsupported>";
    }
  }

  std::shared_ptr<rcpputils::SharedLibrary> lib_;
  std::shared_ptr<rcpputils::SharedLibrary> ilib_;
  const rosidl_message_type_support_t * ts_{nullptr};
  const ti::MessageMembers * members_{nullptr};
};
