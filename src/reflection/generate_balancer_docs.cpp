// generate_balancer_docs.cpp — Reflection-based Markdown docs for balancer_bot IPC.

#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "balancer_message_registry.h"
#include "services/control/control_service.h"
#include "services/imu/imu_service.h"
#include "services/input/input_service.h"
#include "services/motor/motor_service.h"
#include "services/time/time_service.h"
#include "udp_bridge.h"

using namespace balancer_reflection;

namespace {

template <::MsgId Target, ::MsgId... Ids>
consteval bool contains_msg(ipc::MsgList<Ids...>) {
  return ((Target == Ids) || ...);
}

template <typename Component, ::MsgId Id>
consteval bool component_subscribes() {
  if constexpr (requires { typename Component::Subscribes; }) {
    return contains_msg<Id>(typename Component::Subscribes{});
  } else {
    return false;
  }
}

template <typename Component, ::MsgId Id>
consteval bool component_publishes() {
  return contains_msg<Id>(typename Component::Publishes{});
}

template <typename ComponentsT, ::MsgId Id>
consteval bool message_used_by_components() {
  return []<std::size_t... Is>(std::index_sequence<Is...>) {
    return ((component_publishes<std::tuple_element_t<Is, ComponentsT>, Id>() ||
             component_subscribes<std::tuple_element_t<Is, ComponentsT>, Id>()) ||
            ...);
  }(std::make_index_sequence<std::tuple_size_v<ComponentsT>>{});
}

template <typename ComponentsT, typename Fn>
constexpr void for_each_documented_message(Fn&& fn) {
  for_each_message_enum([&]<::MsgId Id>() {
    if constexpr (message_used_by_components<ComponentsT, Id>()) {
      fn.template operator()<Id>();
    }
  });
}

template <typename ComponentsT>
consteval std::size_t documented_message_count() {
  std::size_t count = 0;
  constexpr std::size_t N = get_enum_size<MsgId>();
  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (..., [&] {
      constexpr auto e = EnumArrHolder<MsgId, N>::arr[Is];
      constexpr auto id = static_cast<MsgId>([:e:]);
      if constexpr (message_used_by_components<ComponentsT, id>()) {
        ++count;
      }
    }());
  }(std::make_index_sequence<N>{});
  return count;
}

using Components = std::tuple<sil::ImuService, sil::TimeService, sil::ControlService,
                              sil::MotorService, sil::InputService, ipc::UdpBridge>;

inline std::string dot_escape_label(std::string_view s) {
  std::string out;
  for (char ch : s) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        break;
      default:
        out.push_back(ch);
        break;
    }
  }
  return out;
}

inline std::string html_escape(std::string_view s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      case '\'':
        out += "&apos;";
        break;
      case '\n':
        out += "<BR/>";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

inline std::string dot_id(std::string_view s) {
  std::string out;
  for (char ch : s) {
    unsigned char uc = static_cast<unsigned char>(ch);
    out.push_back(std::isalnum(uc) ? ch : '_');
  }
  if (out.empty() || std::isdigit(static_cast<unsigned char>(out[0]))) {
    out.insert(out.begin(), '_');
  }
  return out;
}

inline std::string first_line(std::string_view s) {
  const std::size_t pos = s.find('\n');
  if (pos == std::string_view::npos) {
    return std::string(s);
  }
  return std::string(s.substr(0, pos));
}

inline std::string wrap_words(std::string_view s, std::size_t width = 40) {
  std::string out;
  std::string word;
  std::size_t col = 0;

  auto flush_word = [&]() {
    if (word.empty()) {
      return;
    }
    if (col == 0) {
      out += word;
      col = word.size();
    } else if (col + 1 + word.size() <= width) {
      out.push_back(' ');
      out += word;
      col += 1 + word.size();
    } else {
      out.push_back('\n');
      out += word;
      col = word.size();
    }
    word.clear();
  };

  for (char ch : s) {
    if (ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t') {
      flush_word();
    } else {
      word.push_back(ch);
    }
  }
  flush_word();
  return out;
}

template <typename T>
concept HasDocDescription = requires { T::kDocDescription; };

template <typename T>
std::string component_doc_text() {
  constexpr auto desc = get_desc<^^T>();
  if constexpr (desc.text[0] != '\0') {
    return desc.text;
  } else if constexpr (HasDocDescription<T>) {
    return T::kDocDescription;
  } else {
    return "";
  }
}

void emit_markdown_blockquote(std::string_view text) {
  if (text.empty()) {
    return;
  }

  std::size_t line_start = 0;
  while (line_start <= text.size()) {
    const std::size_t line_end = text.find('\n', line_start);
    const bool has_newline = (line_end != std::string_view::npos);
    const std::size_t end = has_newline ? line_end : text.size();
    const std::string_view line = text.substr(line_start, end - line_start);

    if (line.empty()) {
      std::cout << ">\n";
    } else {
      std::cout << "> " << line << "\n";
    }

    if (!has_newline) {
      break;
    }
    line_start = line_end + 1;
  }
}

template <typename Tuple, ::MsgId Id, std::size_t... Is>
void print_publishers_impl(std::index_sequence<Is...>) {
  bool first = true;
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_publishes<Component, Id>()) {
      if (!first) {
        std::cout << ", ";
      }
      std::cout << "`" << get_cxx_type_name<Component>() << "`";
      first = false;
    }
  }());
  if (first) {
    std::cout << "_None_";
  }
}

template <typename Tuple, ::MsgId Id>
void print_publishers() {
  print_publishers_impl<Tuple, Id>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
}

template <typename Tuple, ::MsgId Id, std::size_t... Is>
void print_subscribers_impl(std::index_sequence<Is...>) {
  bool first = true;
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_subscribes<Component, Id>()) {
      if (!first) {
        std::cout << ", ";
      }
      std::cout << "`" << get_cxx_type_name<Component>() << "`";
      first = false;
    }
  }());
  if (first) {
    std::cout << "_None_";
  }
}

template <typename Tuple, ::MsgId Id>
void print_subscribers() {
  print_subscribers_impl<Tuple, Id>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
}

template <typename T>
void emit_field_table() {
  constexpr std::size_t N = get_fields_size<T>();

  std::cout << "| Field | C++ Type | Python Type | Bytes | Offset | Description |\n";
  std::cout << "|---|---|---|---:|---:|---|\n";

  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (..., [&] {
      constexpr auto field = StructArrHolder<T, N>::arr[Is];
      constexpr auto type = std::meta::type_of(field);
      using FieldT = typename[:type:];
      constexpr auto field_desc = get_desc<field>();

      std::cout << "| `" << std::meta::identifier_of(field) << "`"
                << " | `" << get_cxx_type_name<FieldT>() << "`"
                << " | `" << get_python_annotation<FieldT>() << "`"
                << " | " << sizeof(FieldT) << " | " << std::meta::offset_of(field).bytes << " | ";
      if (field_desc.text[0] != '\0') {
        std::cout << field_desc.text;
      }
      std::cout << " |\n";
    }());
  }(std::make_index_sequence<N>{});

  std::cout << "\n";
}

template <::MsgId Id>
void emit_payload_section() {
  using Payload = typename MessageTraits<Id>::Payload;
  constexpr auto desc = get_desc<^^Payload>();

  std::cout << "### `MsgId::" << MessageTraits<Id>::name << "`\n\n";
  if (desc.text[0] != '\0') {
    emit_markdown_blockquote(desc.text);
    std::cout << "\n";
  }
  std::cout << "- Numeric ID: `" << static_cast<uint16_t>(Id) << "`\n";
  std::cout << "- Payload type: `" << get_cxx_type_name<Payload>() << "`\n";
  std::cout << "- Python type: `" << get_python_type_name<Payload>() << "`\n";
  std::cout << "- Wire size: `" << sizeof(Payload) << "` bytes\n";
  std::cout << "- Published by: ";
  print_publishers<Components, Id>();
  std::cout << "\n";
  std::cout << "- Consumed by: ";
  print_subscribers<Components, Id>();
  std::cout << "\n\n";

  emit_field_table<Payload>();
}

template <typename Tuple, std::size_t... Is>
void emit_components_impl(std::index_sequence<Is...>) {
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    const std::string doc_text = component_doc_text<Component>();

    std::cout << "### `" << get_cxx_type_name<Component>() << "`\n\n";
    if (!doc_text.empty()) {
      emit_markdown_blockquote(doc_text);
      std::cout << "\n";
    }
    std::cout << "- Publishes: ";
    []<::MsgId... Ids>(ipc::MsgList<Ids...>) {
      bool first = true;
      (((std::cout << (first ? "" : ", ") << "`" << MessageTraits<Ids>::name << "`"),
        first = false),
       ...);
      if (first) {
        std::cout << "_None_";
      }
    }(typename Component::Publishes{});
    std::cout << "\n";

    std::cout << "- Subscribes: ";
    if constexpr (requires { typename Component::Subscribes; }) {
      []<::MsgId... Ids>(ipc::MsgList<Ids...>) {
        bool first = true;
        (((std::cout << (first ? "" : ", ") << "`" << MessageTraits<Ids>::name << "`"),
          first = false),
         ...);
        if (first) {
          std::cout << "_None_";
        }
      }(typename Component::Subscribes{});
    } else {
      std::cout << "_None_";
    }
    std::cout << "\n\n";
  }());
}

void emit_components() {
  emit_components_impl<Components>(std::make_index_sequence<std::tuple_size_v<Components>>{});
}

using EdgeMap = std::map<std::pair<std::string, std::string>, std::set<std::string>>;

template <typename Tuple, ::MsgId Id, std::size_t... Is>
void collect_inbound_edges_impl(EdgeMap& edges, std::index_sequence<Is...>,
                                std::string_view message_name) {
  const std::string bridge = dot_id("UdpBridge");
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_subscribes<Component, Id>() &&
                  !std::is_same_v<Component, ipc::UdpBridge>) {
      edges[{bridge, dot_id(get_cxx_type_name<Component>())}].insert(std::string(message_name));
    }
  }());
}

template <typename Tuple, ::MsgId Id>
void collect_inbound_edges(EdgeMap& edges, std::string_view message_name) {
  collect_inbound_edges_impl<Tuple, Id>(edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{},
                                        message_name);
}

template <typename Tuple, ::MsgId Id, std::size_t... Is>
void collect_outbound_edges_impl(EdgeMap& edges, std::index_sequence<Is...>,
                                 std::string_view message_name) {
  const std::string bridge = dot_id("UdpBridge");
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_publishes<Component, Id>() &&
                  !std::is_same_v<Component, ipc::UdpBridge>) {
      edges[{dot_id(get_cxx_type_name<Component>()), bridge}].insert(std::string(message_name));
    }
  }());
}

template <typename Tuple, ::MsgId Id>
void collect_outbound_edges(EdgeMap& edges, std::string_view message_name) {
  collect_outbound_edges_impl<Tuple, Id>(
      edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{}, message_name);
}

template <typename Tuple, ::MsgId Id, std::size_t... Is>
void collect_internal_edges_impl(EdgeMap& edges, std::index_sequence<Is...>,
                                 std::string_view message_name, std::string_view publisher_name) {
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_subscribes<Component, Id>() &&
                  !std::is_same_v<Component, ipc::UdpBridge>) {
      edges[{dot_id(publisher_name), dot_id(get_cxx_type_name<Component>())}].insert(
          std::string(message_name));
    }
  }());
}

template <typename Tuple, ::MsgId Id, std::size_t... Is>
void collect_internal_edges_pub_impl(EdgeMap& edges, std::index_sequence<Is...>,
                                     std::string_view message_name) {
  (..., [&] {
    using Component = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_publishes<Component, Id>() &&
                  !std::is_same_v<Component, ipc::UdpBridge>) {
      collect_internal_edges_impl<Tuple, Id>(edges,
                                             std::make_index_sequence<std::tuple_size_v<Tuple>>{},
                                             message_name, get_cxx_type_name<Component>());
    }
  }());
}

template <typename Tuple, ::MsgId Id>
void collect_internal_edges(EdgeMap& edges, std::string_view message_name) {
  collect_internal_edges_pub_impl<Tuple, Id>(
      edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{}, message_name);
}

template <typename ComponentsT, ::MsgId Id>
void collect_msg_edges(EdgeMap& edges, std::set<std::string>& inbound_message_names,
                       std::set<std::string>& outbound_message_names) {
  constexpr bool has_cpp_subscribers = []<std::size_t... Is>(std::index_sequence<Is...>) {
    return ((component_subscribes<std::tuple_element_t<Is, ComponentsT>, Id>() &&
             !std::is_same_v<std::tuple_element_t<Is, ComponentsT>, ipc::UdpBridge>) ||
            ...);
  }(std::make_index_sequence<std::tuple_size_v<ComponentsT>>{});

  constexpr bool has_cpp_publishers = []<std::size_t... Is>(std::index_sequence<Is...>) {
    return ((component_publishes<std::tuple_element_t<Is, ComponentsT>, Id>() &&
             !std::is_same_v<std::tuple_element_t<Is, ComponentsT>, ipc::UdpBridge>) ||
            ...);
  }(std::make_index_sequence<std::tuple_size_v<ComponentsT>>{});

  constexpr bool bridge_subscribes = component_subscribes<ipc::UdpBridge, Id>();
  constexpr bool bridge_publishes = component_publishes<ipc::UdpBridge, Id>();
  constexpr std::string_view message_name = MessageTraits<Id>::name;

  if constexpr (bridge_publishes && has_cpp_subscribers) {
    inbound_message_names.insert(std::string(message_name));
    collect_inbound_edges<ComponentsT, Id>(edges, message_name);
  }

  if constexpr (has_cpp_publishers && bridge_subscribes) {
    outbound_message_names.insert(std::string(message_name));
    collect_outbound_edges<ComponentsT, Id>(edges, message_name);
  }

  if constexpr (has_cpp_publishers && has_cpp_subscribers) {
    collect_internal_edges<ComponentsT, Id>(edges, message_name);
  }
}

inline std::string build_label(const std::set<std::string>& names) {
  std::string out;
  int count = 0;
  for (const auto& name : names) {
    if (count > 0) {
      out += (count % 2 == 0) ? "\n" : ", ";
    }
    out += name;
    ++count;
  }
  return out;
}

void emit_graphviz_flow_dot(std::ostream& os) {
  os << "digraph IPC {\n";
  os << "  rankdir=LR;\n";
  os << "  bgcolor=\"#0F172A\";\n";
  os << "  pad=0.22;\n";
  os << "  nodesep=0.62;\n";
  os << "  ranksep=0.70;\n";
  os << "  splines=spline;\n";
  os << "  remincross=true;\n\n";
  os << "  fontname=\"Helvetica,Arial,sans-serif\";\n";
  os << "  fontsize=52;\n";
  os << "  fontcolor=\"#F1F5F9\";\n";
  os << "  label=<<B>balancer_bot IPC Flow</B>>;\n";
  os << "  labelloc=\"t\";\n\n";
  os << "  node [fontname=\"Helvetica,Arial,sans-serif\", shape=rect, style=\"filled,rounded\", "
        "fixedsize=false, margin=\"0.12,0.08\", fontsize=28, fontcolor=\"#FFFFFF\", penwidth=4, "
        "color=\"#F1F5F9\"];\n\n";
  os << "  edge [fontname=\"Helvetica,Arial,sans-serif\", fontsize=22, fontcolor=\"#CBD5E1\", "
        "color=\"#64748B\", penwidth=3.0, arrowsize=1.4, labelfloat=false, labeldistance=0.8, "
        "labelangle=10];\n\n";

  os << "  subgraph cluster_sim {\n";
  os << "    label=<<B><FONT POINT-SIZE=\"40\">Balancer Services</FONT></B>>;\n";
  os << "    fontcolor=\"#F1F5F9\";\n";
  os << "    style=\"rounded,filled\";\n";
  os << "    color=\"#475569\";\n";
  os << "    penwidth=4;\n";
  os << "    fillcolor=\"#1E293B\";\n";
  os << "    margin=34;\n";

  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (..., [&] {
      using Component = std::tuple_element_t<Is, Components>;
      const std::string component_name = get_cxx_type_name<Component>();
      const std::string doc_text = component_doc_text<Component>();
      std::string summary = wrap_words(first_line(doc_text), 38);
      if (summary.empty()) {
        summary = component_name;
      }

      const bool is_udp_bridge = std::is_same_v<Component, ipc::UdpBridge>;
      os << "    " << dot_id(component_name) << " [\n";
      os << "      fillcolor=\"" << (is_udp_bridge ? "#0F766E" : "#0369A1") << "\",\n";
      os << "      label=<<B><FONT POINT-SIZE=\"" << (is_udp_bridge ? "38" : "30") << "\">"
         << html_escape(component_name) << "</FONT></B><BR/><FONT POINT-SIZE=\"24\">"
         << html_escape(summary) << "</FONT>>\n";
      os << "    ];\n";
    }());
  }(std::make_index_sequence<std::tuple_size_v<Components>>{});
  os << "  }\n\n";

  os << "  subgraph cluster_sockets {\n";
  os << "    label=<<B><FONT POINT-SIZE=\"40\">Network Layer</FONT></B>>;\n";
  os << "    fontcolor=\"#F1F5F9\";\n";
  os << "    style=\"rounded,filled\";\n";
  os << "    color=\"#F59E0B\";\n";
  os << "    penwidth=6;\n";
  os << "    fillcolor=\"#2A1B07\";\n";
  os << "    margin=24;\n\n";
  os << "    { rank=same; TX; RX; }\n\n";
  os << "    TX [fillcolor=\"#B45309\", label=<<B><FONT POINT-SIZE=\"30\">TX Socket</FONT></B>"
        "<BR/><FONT POINT-SIZE=\"24\">Port 9000 (Inbound)</FONT>>];\n";
  os << "    RX [fillcolor=\"#B45309\", label=<<B><FONT POINT-SIZE=\"30\">RX Socket</FONT></B>"
        "<BR/><FONT POINT-SIZE=\"24\">Client Ephemeral Port (Outbound)</FONT>>];\n";
  os << "    TX -> RX [style=invis, weight=50, constraint=false];\n";
  os << "  }\n\n";

  os << "  subgraph cluster_pytest {\n";
  os << "    label=<<B><FONT POINT-SIZE=\"40\">Pytest Harness</FONT></B>>;\n";
  os << "    fontcolor=\"#F1F5F9\";\n";
  os << "    style=\"rounded,filled\";\n";
  os << "    color=\"#22C55E\";\n";
  os << "    penwidth=6;\n";
  os << "    fillcolor=\"#0B2A20\";\n";
  os << "    margin=24;\n\n";
  os << "    TestCase [fillcolor=\"#047857\", label=<<B><FONT POINT-SIZE=\"34\">Test Case / "
        "Fixtures</FONT></B>>];\n";
  os << "  }\n\n";

  EdgeMap edge_map;
  std::set<std::string> inbound_message_names;
  std::set<std::string> outbound_message_names;
  for_each_documented_message<Components>([&]<::MsgId Id>() {
    collect_msg_edges<Components, Id>(edge_map, inbound_message_names, outbound_message_names);
  });

  os << "  TestCase -> TX [label=\"send_msg\", color=\"#F1F5F9\", penwidth=5];\n";
  os << "  TX -> " << dot_id("UdpBridge") << " [label=\"Inbound Traffic:\\n"
     << dot_escape_label(build_label(inbound_message_names))
     << "\", color=\"#F1F5F9\", penwidth=5, fontsize=25];\n\n";
  os << "  // Internal Service Logic\n";
  for (const auto& [edge, names] : edge_map) {
    os << "  " << edge.first << " -> " << edge.second << " [label=\""
       << dot_escape_label(build_label(names)) << "\", style=dashed];\n";
  }
  os << "\n  RX -> " << dot_id("UdpBridge") << " [label=\"Outbound Traffic:\\n"
     << dot_escape_label(build_label(outbound_message_names))
     << "\", color=\"#F1F5F9\", penwidth=5, dir=back, fontsize=25, arrowtail=normal];\n";
  os << "  TestCase -> RX [label=\"recv_msg\", color=\"#F1F5F9\", penwidth=5, dir=back, "
        "arrowtail=normal];\n";
  os << "}\n";
}

void write_graphviz_flow_dot_file(std::string_view path) {
  std::ofstream out{std::string(path)};
  if (!out) {
    std::cerr << "generate_balancer_docs: failed to open DOT output: " << path << "\n";
    return;
  }
  emit_graphviz_flow_dot(out);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1) {
    write_graphviz_flow_dot_file(argv[1]);
  }

  std::cout << R"(# IPC Protocol Reference

[Home](../../README.md)

> **Auto-generated** by `generate_balancer_docs` using C++26 static reflection (P2996 + P3394).
> Do not edit by hand. Re-run `cmake --build build --target balancer_docs` to refresh.

## Overview

This document is generated from the balancer runtime message registry and the reflected payload types in `src/messages/`.
It describes the reflected runtime message bus used by the balancer services, including the UDP-facing
messages consumed by the SIL harness and the internal-only messages exchanged between services.

)";

  std::cout << "- Documented balancer message count: `" << documented_message_count<Components>()
            << "`\n";
  std::cout << "- Protocol hash: `" << compute_protocol_hash() << "`\n";
  std::cout << "- UDP ingress/egress gateway: `UdpBridge`\n\n";

  std::cout << R"(## System Architecture

The architecture is divided into three logical areas:

1. **Pytest Harness**: Python fixtures send fixed-size UDP datagrams and decode telemetry using the generated bindings.
2. **Network Layer**: `UdpBridge` translates between UDP traffic and the internal message bus.
3. **Balancer Services**: services publish and consume reflected payload structs on the bus.

![IPC Flow Diagram](ipc_flow.svg)

---

## Component Services

)";

  emit_components();

  std::cout << R"(---

## Message Payloads

Each section corresponds to one reflected balancer message. Some are exposed over UDP, while others are
internal-only service messages. Wire sizes come directly from `sizeof(Payload)`.

)";

  for_each_documented_message<Components>([&]<::MsgId Id>() { emit_payload_section<Id>(); });

  std::cout << R"(---

## Regenerating This File

```bash
cmake -S . -B build
cmake --build build --target balancer_docs
```

_Generated with GCC trunk `-std=c++26 -freflection`._
)";

  return 0;
}
