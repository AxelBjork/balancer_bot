// generate_balancer_bindings.cpp — C++26 reflection generator for balancer_bot payloads.

#include <iostream>
#include <set>
#include <string>
#include <type_traits>
#include <utility>

#include "balancer_message_registry.h"

using namespace balancer_reflection;

template <typename E>
void generate_enum(std::set<std::string>& visited) {
  const std::string class_name{std::meta::identifier_of(^^E)};
  if (!visited.insert(class_name).second) {
    return;
  }

  std::cout << "class " << class_name << "(IntEnum):\n";
  constexpr auto desc = get_desc<^^E>();
  if (desc.text[0] != '\0') {
    std::cout << "    \"\"\"" << desc.text << "\"\"\"\n";
  }

  using UnderlyingT = std::underlying_type_t<E>;
  template for (constexpr auto e : reflected_enumerators<E>()) {
    std::cout << "    " << std::meta::identifier_of(e) << " = "
              << static_cast<UnderlyingT>([:e:]) << "\n";
  }

  std::cout << "\n";
}

void generate_balancer_msg_enum() {
  std::cout << "class MsgId(IntEnum):\n";
  std::cout << "    \"\"\"Balancer UDP message identifiers.\"\"\"\n";
  for_each_udp_message([&]<::MsgId Id>() {
    std::cout << "    " << get_enum_name<MsgId, Id>() << " = " << static_cast<uint16_t>(Id)
              << "\n";
  });
  std::cout << "\n";
  std::cout << "BalancerMsgId = MsgId\n\n";
}

template <typename T>
void generate_struct(std::set<std::string>& visited, std::set<std::string>& visited_enums) {
  validate_wire_type<T>();
  std::string class_name = get_python_type_name<T>();
  if (visited.count(class_name)) {
    return;
  }
  visited.insert(class_name);

  template for (constexpr auto field : reflected_members<T>()) {
    constexpr auto type = std::meta::type_of(field);
    using FieldT = typename[:type:];
    using BaseT = std::remove_all_extents_t<FieldT>;

    if constexpr (is_array_like_v<FieldT>) {
      using ArrayT = std::remove_cv_t<FieldT>;
      using ElemT = std::remove_cv_t<typename array_traits<ArrayT>::element_type>;
      if constexpr (std::is_enum_v<ElemT>) {
        generate_enum<ElemT>(visited_enums);
      } else if constexpr (std::is_class_v<ElemT> && !is_array_like_v<ElemT>) {
        generate_struct<ElemT>(visited, visited_enums);
      }
    } else if constexpr (std::is_enum_v<BaseT>) {
      generate_enum<std::remove_cv_t<BaseT>>(visited_enums);
    } else if constexpr (std::is_class_v<BaseT>) {
      generate_struct<BaseT>(visited, visited_enums);
    }
  }

  std::cout << "@dataclass\nclass " << class_name << ":\n";
  constexpr auto desc = get_desc<^^T>();
  if (desc.text[0] != '\0') {
    std::cout << "    \"\"\"" << desc.text << "\"\"\"\n";
  }
  std::cout << "    WIRE_SIZE = " << sizeof(T) << "\n";

  std::string pack_fmt = "<";
  std::string pack_args;
  std::string unpack_args;
  std::string pack_instructions = "        data = bytearray()\n";
  std::string unpack_instructions = "        offset = 0\n";
  std::size_t current_offset = 0;

  auto flush_format = [&]() {
    if (pack_fmt == "<") {
      return;
    }

    pack_instructions +=
        "        data.extend(struct.pack(\"" + pack_fmt + "\", " + pack_args + "))\n";

    const bool is_single_value = (unpack_args.find(',') == std::string::npos);
    if (is_single_value) {
      unpack_instructions += "        " + unpack_args + " = struct.unpack_from(\"" + pack_fmt +
                             "\", data, offset)[0]\n";
    } else {
      unpack_instructions +=
          "        " + unpack_args + " = struct.unpack_from(\"" + pack_fmt + "\", data, offset)\n";
    }
    unpack_instructions += "        offset += struct.calcsize(\"" + pack_fmt + "\")\n";

    pack_fmt = "<";
    pack_args.clear();
    unpack_args.clear();
  };

  if constexpr (reflected_members<T>().size() > 0) {
    template for (constexpr auto field : reflected_members<T>()) {
      constexpr auto type = std::meta::type_of(field);
      using FieldT = typename[:type:];

      std::string field_name{std::meta::identifier_of(field)};
      std::cout << "    " << field_name << ": " << get_python_annotation<FieldT>() << "\n";

      constexpr auto field_offset = std::meta::offset_of(field).bytes;
      if (field_offset > current_offset) {
        const std::size_t pad = field_offset - current_offset;
        pack_fmt += std::to_string(pad) + "x";
        current_offset += pad;
      }

      if constexpr (is_array_like_v<FieldT>) {
        using ArrayT = std::remove_cv_t<FieldT>;
        using ElemT = std::remove_cv_t<typename array_traits<ArrayT>::element_type>;
        constexpr std::size_t elem_count = array_traits<ArrayT>::size;

        if constexpr (is_byte_like_v<ElemT>) {
          pack_fmt += std::to_string(elem_count) + "s";
          if (!pack_args.empty()) {
            pack_args += ", ";
          }
          pack_args += "self." + field_name;

          if (!unpack_args.empty()) {
            unpack_args += ", ";
          }
          unpack_args += field_name;
          current_offset += elem_count;
        } else if constexpr (std::is_class_v<ElemT>) {
          flush_format();
          const std::string sub_type = get_python_type_name<ElemT>();
          pack_instructions += "        for item in self." + field_name + ":\n";
          pack_instructions += "            if not hasattr(item, 'pack_wire'):\n";
          pack_instructions += "                if isinstance(item, tuple):\n";
          pack_instructions += "                    item = " + sub_type + "(*item)\n";
          pack_instructions += "                elif isinstance(item, dict):\n";
          pack_instructions += "                    item = " + sub_type + "(**item)\n";
          pack_instructions += "                else:\n";
          pack_instructions += "                    item = " + sub_type + "(item)\n";
          pack_instructions += "            data.extend(item.pack_wire())\n";

          unpack_instructions += "        " + field_name + " = []\n";
          unpack_instructions += "        for _ in range(" + std::to_string(elem_count) + "):\n";
          unpack_instructions += "            sub_size = " + sub_type + ".WIRE_SIZE\n";
          unpack_instructions +=
              "            item = " + sub_type + ".unpack_wire(data[offset:offset+sub_size])\n";
          unpack_instructions += "            " + field_name + ".append(item)\n";
          unpack_instructions += "            offset += sub_size\n";
          current_offset += elem_count * sizeof(ElemT);
        } else {
          flush_format();
          const std::string fmt =
              "<" + std::to_string(elem_count) + std::string(get_struct_format_char<^^ElemT>());
          pack_instructions +=
              "        data.extend(struct.pack(\"" + fmt + "\", *self." + field_name + "))\n";
          unpack_instructions += "        " + field_name + " = list(struct.unpack_from(\"" + fmt +
                                 "\", data, offset))\n";
          unpack_instructions += "        offset += struct.calcsize(\"" + fmt + "\")\n";
          current_offset += elem_count * sizeof(ElemT);
        }
      } else if constexpr (std::is_class_v<FieldT>) {
        flush_format();
        const std::string sub_type = get_python_type_name<FieldT>();
        pack_instructions += "        item = self." + field_name + "\n";
        pack_instructions += "        if not hasattr(item, 'pack_wire'):\n";
        pack_instructions += "            if isinstance(item, tuple):\n";
        pack_instructions += "                item = " + sub_type + "(*item)\n";
        pack_instructions += "            elif isinstance(item, dict):\n";
        pack_instructions += "                item = " + sub_type + "(**item)\n";
        pack_instructions += "            else:\n";
        pack_instructions += "                item = " + sub_type + "(item)\n";
        pack_instructions += "        data.extend(item.pack_wire())\n";

        unpack_instructions += "        sub_size = " + sub_type + ".WIRE_SIZE\n";
        unpack_instructions += "        " + field_name + " = " + sub_type +
                               ".unpack_wire(data[offset:offset+sub_size])\n";
        unpack_instructions += "        offset += sub_size\n";
        current_offset += sizeof(FieldT);
      } else {
        pack_fmt += get_struct_format_char<type>();
        if (!pack_args.empty()) {
          pack_args += ", ";
        }
        pack_args += "self." + field_name;

        if (!unpack_args.empty()) {
          unpack_args += ", ";
        }
        unpack_args += field_name;
        current_offset += sizeof(FieldT);
      }
    }

    if (sizeof(T) > current_offset) {
      const std::size_t pad = sizeof(T) - current_offset;
      pack_fmt += std::to_string(pad) + "x";
      current_offset += pad;
    }

    flush_format();
  } else {
    std::cout << "    pass\n";
  }

  std::cout << "\n";
  std::cout << "    def pack_wire(self) -> bytes:\n";
  if constexpr (reflected_members<T>().size() > 0) {
    std::cout << pack_instructions;
    std::cout << "        return bytes(data)\n\n";
  } else {
    std::cout << "        return b\"\"\n\n";
  }

  std::cout << "    def pack(self) -> bytes:\n";
  std::cout << "        return self.pack_wire()\n\n";

  std::cout << "    @classmethod\n";
  std::cout << "    def unpack_wire(cls, data: bytes) -> \"" << class_name << "\":\n";
  if constexpr (reflected_members<T>().size() > 0) {
    std::cout << unpack_instructions;
    std::cout << "        return cls(";

    bool first = true;
    template for (constexpr auto field : reflected_members<T>()) {
      std::string field_name{std::meta::identifier_of(field)};
      if (!first) {
        std::cout << ", ";
      }
      std::cout << field_name << "=" << field_name;
      first = false;
    }

    std::cout << ")\n\n";
  } else {
    std::cout << "        return cls()\n\n";
  }

  std::cout << "    @classmethod\n";
  std::cout << "    def unpack(cls, data: bytes) -> \"" << class_name << "\":\n";
  std::cout << "        return cls.unpack_wire(data)\n\n";
}

int main() {
  std::cout << "\"\"\"Auto-generated IPC bindings using C++26 static reflection.\"\"\"\n\n";
  std::cout << "import struct\n";
  std::cout << "from dataclasses import dataclass\n";
  std::cout << "from enum import IntEnum\n";
  std::cout << "from typing import Any\n\n";

  generate_balancer_msg_enum();

  std::set<std::string> visited_structs;
  std::set<std::string> visited_enums{"MsgId"};
  for_each_udp_message(
      [&]<::MsgId Id>() {
        generate_struct<typename MessageTraits<Id>::Payload>(visited_structs, visited_enums);
      });

  std::cout << "MESSAGE_BY_ID = {\n";
  for_each_udp_message([&]<::MsgId Id>() {
    std::cout << "    MsgId." << get_enum_name<MsgId, Id>() << ": "
              << get_python_type_name<typename MessageTraits<Id>::Payload>() << ",\n";
  });
  std::cout << "}\n\n";

  std::cout << "PAYLOAD_SIZE_BY_ID = {\n";
  for_each_udp_message([&]<::MsgId Id>() {
    std::cout << "    MsgId." << get_enum_name<MsgId, Id>() << ": "
              << sizeof(typename MessageTraits<Id>::Payload) << ",\n";
  });
  std::cout << "}\n\n";

  std::cout << "PROTOCOL_HASH = \"" << compute_protocol_hash() << "\"\n";
  return 0;
}
