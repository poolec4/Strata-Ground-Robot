// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vicon.proto

#ifndef PROTOBUF_vicon_2eproto__INCLUDED
#define PROTOBUF_vicon_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_vicon_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsrobot_state_requestImpl();
void InitDefaultsrobot_state_request();
void InitDefaultsrobot_state_replyImpl();
void InitDefaultsrobot_state_reply();
inline void InitDefaults() {
  InitDefaultsrobot_state_request();
  InitDefaultsrobot_state_reply();
}
}  // namespace protobuf_vicon_2eproto
namespace vicon {
class robot_state_reply;
class robot_state_replyDefaultTypeInternal;
extern robot_state_replyDefaultTypeInternal _robot_state_reply_default_instance_;
class robot_state_request;
class robot_state_requestDefaultTypeInternal;
extern robot_state_requestDefaultTypeInternal _robot_state_request_default_instance_;
}  // namespace vicon
namespace vicon {

// ===================================================================

class robot_state_request : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:vicon.robot_state_request) */ {
 public:
  robot_state_request();
  virtual ~robot_state_request();

  robot_state_request(const robot_state_request& from);

  inline robot_state_request& operator=(const robot_state_request& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  robot_state_request(robot_state_request&& from) noexcept
    : robot_state_request() {
    *this = ::std::move(from);
  }

  inline robot_state_request& operator=(robot_state_request&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const robot_state_request& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const robot_state_request* internal_default_instance() {
    return reinterpret_cast<const robot_state_request*>(
               &_robot_state_request_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(robot_state_request* other);
  friend void swap(robot_state_request& a, robot_state_request& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline robot_state_request* New() const PROTOBUF_FINAL { return New(NULL); }

  robot_state_request* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const robot_state_request& from);
  void MergeFrom(const robot_state_request& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(robot_state_request* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // string name = 1;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  #if LANG_CXX11
  void set_name(::std::string&& value);
  #endif
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // @@protoc_insertion_point(class_scope:vicon.robot_state_request)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  mutable int _cached_size_;
  friend struct ::protobuf_vicon_2eproto::TableStruct;
  friend void ::protobuf_vicon_2eproto::InitDefaultsrobot_state_requestImpl();
};
// -------------------------------------------------------------------

class robot_state_reply : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:vicon.robot_state_reply) */ {
 public:
  robot_state_reply();
  virtual ~robot_state_reply();

  robot_state_reply(const robot_state_reply& from);

  inline robot_state_reply& operator=(const robot_state_reply& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  robot_state_reply(robot_state_reply&& from) noexcept
    : robot_state_reply() {
    *this = ::std::move(from);
  }

  inline robot_state_reply& operator=(robot_state_reply&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const robot_state_reply& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const robot_state_reply* internal_default_instance() {
    return reinterpret_cast<const robot_state_reply*>(
               &_robot_state_reply_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(robot_state_reply* other);
  friend void swap(robot_state_reply& a, robot_state_reply& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline robot_state_reply* New() const PROTOBUF_FINAL { return New(NULL); }

  robot_state_reply* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const robot_state_reply& from);
  void MergeFrom(const robot_state_reply& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(robot_state_reply* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // float tx = 1;
  void clear_tx();
  static const int kTxFieldNumber = 1;
  float tx() const;
  void set_tx(float value);

  // float ty = 2;
  void clear_ty();
  static const int kTyFieldNumber = 2;
  float ty() const;
  void set_ty(float value);

  // float tz = 3;
  void clear_tz();
  static const int kTzFieldNumber = 3;
  float tz() const;
  void set_tz(float value);

  // float ox = 4;
  void clear_ox();
  static const int kOxFieldNumber = 4;
  float ox() const;
  void set_ox(float value);

  // float oy = 5;
  void clear_oy();
  static const int kOyFieldNumber = 5;
  float oy() const;
  void set_oy(float value);

  // float oz = 6;
  void clear_oz();
  static const int kOzFieldNumber = 6;
  float oz() const;
  void set_oz(float value);

  // float ow = 7;
  void clear_ow();
  static const int kOwFieldNumber = 7;
  float ow() const;
  void set_ow(float value);

  // @@protoc_insertion_point(class_scope:vicon.robot_state_reply)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  float tx_;
  float ty_;
  float tz_;
  float ox_;
  float oy_;
  float oz_;
  float ow_;
  mutable int _cached_size_;
  friend struct ::protobuf_vicon_2eproto::TableStruct;
  friend void ::protobuf_vicon_2eproto::InitDefaultsrobot_state_replyImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// robot_state_request

// string name = 1;
inline void robot_state_request::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& robot_state_request::name() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_request.name)
  return name_.GetNoArena();
}
inline void robot_state_request::set_name(const ::std::string& value) {
  
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:vicon.robot_state_request.name)
}
#if LANG_CXX11
inline void robot_state_request::set_name(::std::string&& value) {
  
  name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:vicon.robot_state_request.name)
}
#endif
inline void robot_state_request::set_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:vicon.robot_state_request.name)
}
inline void robot_state_request::set_name(const char* value, size_t size) {
  
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:vicon.robot_state_request.name)
}
inline ::std::string* robot_state_request::mutable_name() {
  
  // @@protoc_insertion_point(field_mutable:vicon.robot_state_request.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* robot_state_request::release_name() {
  // @@protoc_insertion_point(field_release:vicon.robot_state_request.name)
  
  return name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void robot_state_request::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    
  } else {
    
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:vicon.robot_state_request.name)
}

// -------------------------------------------------------------------

// robot_state_reply

// float tx = 1;
inline void robot_state_reply::clear_tx() {
  tx_ = 0;
}
inline float robot_state_reply::tx() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.tx)
  return tx_;
}
inline void robot_state_reply::set_tx(float value) {
  
  tx_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.tx)
}

// float ty = 2;
inline void robot_state_reply::clear_ty() {
  ty_ = 0;
}
inline float robot_state_reply::ty() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.ty)
  return ty_;
}
inline void robot_state_reply::set_ty(float value) {
  
  ty_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.ty)
}

// float tz = 3;
inline void robot_state_reply::clear_tz() {
  tz_ = 0;
}
inline float robot_state_reply::tz() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.tz)
  return tz_;
}
inline void robot_state_reply::set_tz(float value) {
  
  tz_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.tz)
}

// float ox = 4;
inline void robot_state_reply::clear_ox() {
  ox_ = 0;
}
inline float robot_state_reply::ox() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.ox)
  return ox_;
}
inline void robot_state_reply::set_ox(float value) {
  
  ox_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.ox)
}

// float oy = 5;
inline void robot_state_reply::clear_oy() {
  oy_ = 0;
}
inline float robot_state_reply::oy() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.oy)
  return oy_;
}
inline void robot_state_reply::set_oy(float value) {
  
  oy_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.oy)
}

// float oz = 6;
inline void robot_state_reply::clear_oz() {
  oz_ = 0;
}
inline float robot_state_reply::oz() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.oz)
  return oz_;
}
inline void robot_state_reply::set_oz(float value) {
  
  oz_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.oz)
}

// float ow = 7;
inline void robot_state_reply::clear_ow() {
  ow_ = 0;
}
inline float robot_state_reply::ow() const {
  // @@protoc_insertion_point(field_get:vicon.robot_state_reply.ow)
  return ow_;
}
inline void robot_state_reply::set_ow(float value) {
  
  ow_ = value;
  // @@protoc_insertion_point(field_set:vicon.robot_state_reply.ow)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace vicon

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_vicon_2eproto__INCLUDED