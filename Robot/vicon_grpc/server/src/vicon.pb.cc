// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vicon.proto

#include "vicon.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
class robot_stateDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<robot_state>
      _instance;
} _robot_state_default_instance_;
namespace protobuf_vicon_2eproto {
void InitDefaultsrobot_stateImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  {
    void* ptr = &::_robot_state_default_instance_;
    new (ptr) ::robot_state();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::robot_state::InitAsDefaultInstance();
}

void InitDefaultsrobot_state() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsrobot_stateImpl);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, tx_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, ty_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, tz_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, ox_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, oy_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, oz_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::robot_state, ow_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::robot_state)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::_robot_state_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "vicon.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\013vicon.proto\"a\n\013robot_state\022\n\n\002tx\030\001 \001(\002"
      "\022\n\n\002ty\030\002 \001(\002\022\n\n\002tz\030\003 \001(\002\022\n\n\002ox\030\004 \001(\002\022\n\n\002"
      "oy\030\005 \001(\002\022\n\n\002oz\030\006 \001(\002\022\n\n\002ow\030\007 \001(\002b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 120);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "vicon.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_vicon_2eproto

// ===================================================================

void robot_state::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int robot_state::kTxFieldNumber;
const int robot_state::kTyFieldNumber;
const int robot_state::kTzFieldNumber;
const int robot_state::kOxFieldNumber;
const int robot_state::kOyFieldNumber;
const int robot_state::kOzFieldNumber;
const int robot_state::kOwFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

robot_state::robot_state()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_vicon_2eproto::InitDefaultsrobot_state();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:robot_state)
}
robot_state::robot_state(const robot_state& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&tx_, &from.tx_,
    static_cast<size_t>(reinterpret_cast<char*>(&ow_) -
    reinterpret_cast<char*>(&tx_)) + sizeof(ow_));
  // @@protoc_insertion_point(copy_constructor:robot_state)
}

void robot_state::SharedCtor() {
  ::memset(&tx_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&ow_) -
      reinterpret_cast<char*>(&tx_)) + sizeof(ow_));
  _cached_size_ = 0;
}

robot_state::~robot_state() {
  // @@protoc_insertion_point(destructor:robot_state)
  SharedDtor();
}

void robot_state::SharedDtor() {
}

void robot_state::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* robot_state::descriptor() {
  ::protobuf_vicon_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vicon_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const robot_state& robot_state::default_instance() {
  ::protobuf_vicon_2eproto::InitDefaultsrobot_state();
  return *internal_default_instance();
}

robot_state* robot_state::New(::google::protobuf::Arena* arena) const {
  robot_state* n = new robot_state;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void robot_state::Clear() {
// @@protoc_insertion_point(message_clear_start:robot_state)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&tx_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&ow_) -
      reinterpret_cast<char*>(&tx_)) + sizeof(ow_));
  _internal_metadata_.Clear();
}

bool robot_state::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:robot_state)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float tx = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &tx_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float ty = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &ty_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float tz = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &tz_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float ox = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(37u /* 37 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &ox_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float oy = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(45u /* 45 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &oy_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float oz = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(53u /* 53 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &oz_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float ow = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(61u /* 61 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &ow_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:robot_state)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:robot_state)
  return false;
#undef DO_
}

void robot_state::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:robot_state)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float tx = 1;
  if (this->tx() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->tx(), output);
  }

  // float ty = 2;
  if (this->ty() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->ty(), output);
  }

  // float tz = 3;
  if (this->tz() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->tz(), output);
  }

  // float ox = 4;
  if (this->ox() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->ox(), output);
  }

  // float oy = 5;
  if (this->oy() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(5, this->oy(), output);
  }

  // float oz = 6;
  if (this->oz() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(6, this->oz(), output);
  }

  // float ow = 7;
  if (this->ow() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(7, this->ow(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:robot_state)
}

::google::protobuf::uint8* robot_state::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:robot_state)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float tx = 1;
  if (this->tx() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->tx(), target);
  }

  // float ty = 2;
  if (this->ty() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->ty(), target);
  }

  // float tz = 3;
  if (this->tz() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->tz(), target);
  }

  // float ox = 4;
  if (this->ox() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->ox(), target);
  }

  // float oy = 5;
  if (this->oy() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(5, this->oy(), target);
  }

  // float oz = 6;
  if (this->oz() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(6, this->oz(), target);
  }

  // float ow = 7;
  if (this->ow() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(7, this->ow(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:robot_state)
  return target;
}

size_t robot_state::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:robot_state)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // float tx = 1;
  if (this->tx() != 0) {
    total_size += 1 + 4;
  }

  // float ty = 2;
  if (this->ty() != 0) {
    total_size += 1 + 4;
  }

  // float tz = 3;
  if (this->tz() != 0) {
    total_size += 1 + 4;
  }

  // float ox = 4;
  if (this->ox() != 0) {
    total_size += 1 + 4;
  }

  // float oy = 5;
  if (this->oy() != 0) {
    total_size += 1 + 4;
  }

  // float oz = 6;
  if (this->oz() != 0) {
    total_size += 1 + 4;
  }

  // float ow = 7;
  if (this->ow() != 0) {
    total_size += 1 + 4;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void robot_state::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:robot_state)
  GOOGLE_DCHECK_NE(&from, this);
  const robot_state* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const robot_state>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:robot_state)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:robot_state)
    MergeFrom(*source);
  }
}

void robot_state::MergeFrom(const robot_state& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:robot_state)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.tx() != 0) {
    set_tx(from.tx());
  }
  if (from.ty() != 0) {
    set_ty(from.ty());
  }
  if (from.tz() != 0) {
    set_tz(from.tz());
  }
  if (from.ox() != 0) {
    set_ox(from.ox());
  }
  if (from.oy() != 0) {
    set_oy(from.oy());
  }
  if (from.oz() != 0) {
    set_oz(from.oz());
  }
  if (from.ow() != 0) {
    set_ow(from.ow());
  }
}

void robot_state::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:robot_state)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void robot_state::CopyFrom(const robot_state& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:robot_state)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool robot_state::IsInitialized() const {
  return true;
}

void robot_state::Swap(robot_state* other) {
  if (other == this) return;
  InternalSwap(other);
}
void robot_state::InternalSwap(robot_state* other) {
  using std::swap;
  swap(tx_, other->tx_);
  swap(ty_, other->ty_);
  swap(tz_, other->tz_);
  swap(ox_, other->ox_);
  swap(oy_, other->oy_);
  swap(oz_, other->oz_);
  swap(ow_, other->ow_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata robot_state::GetMetadata() const {
  protobuf_vicon_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vicon_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)

// @@protoc_insertion_point(global_scope)