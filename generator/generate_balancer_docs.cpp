// generate_balancer_docs.cpp — Reflection-based Markdown docs for balancer_bot IPC.

#include <array>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <meta>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>

#include "msg_base.h"
#include "udp_bridge.h"
#include "balancer_msgs.h"

using namespace std::string_view_literals;

namespace balancer_doc {

struct DOC_DESC("Publishes fused IMU samples onto the internal message bus. In SIL mode the hardware reader is disabled and samples are injected over UDP instead.") ImuService {
  using Publishes = ipc::MsgList<ipc::ImuData>;
  using Subscribes = ipc::MsgList<>;
};

struct DOC_DESC("Consumes IMU and joystick inputs, runs the cascaded balancing controller, and publishes motor targets plus lightweight telemetry.") ControlService {
  using Publishes = ipc::MsgList<ipc::MotorTargets, ipc::SystemTelemetry>;
  using Subscribes = ipc::MsgList<ipc::ImuData, ipc::JoystickCommand>;
};

struct DOC_DESC("Consumes wheel-speed targets and forwards them to the motor runner when hardware is attached.") MotorService {
  using Publishes = ipc::MsgList<>;
  using Subscribes = ipc::MsgList<ipc::MotorTargets>;
};

}  // namespace balancer_doc

namespace {

template <std::meta::info R>
consteval doc::Desc get_desc() {
  for (std::meta::info a : std::meta::annotations_of(R)) {
    if (std::meta::type_of(a) == ^^doc::Desc) {
      return std::meta::extract<doc::Desc>(a);
    }
    if (std::meta::type_of(a) == ^^const doc::Desc) {
      return std::meta::extract<const doc::Desc>(a);
    }
  }
  return doc::Desc("");
}

template <typename T>
consteval std::size_t get_fields_size() {
  auto ctx = std::meta::access_context::current();
  return std::meta::nonstatic_data_members_of(^^T, ctx).size();
}

template <typename T, std::size_t N>
consteval std::array<std::meta::info, N> get_fields_array() {
  auto ctx = std::meta::access_context::current();
  auto vec = std::meta::nonstatic_data_members_of(^^T, ctx);
  std::array<std::meta::info, N> arr{};
  for (std::size_t i = 0; i < N; ++i) {
    arr[i] = vec[i];
  }
  return arr;
}

template <typename T, std::size_t N>
struct StructArrHolder {
  static constexpr auto arr = get_fields_array<T, N>();
};

template <typename T>
std::string cpp_type_name_str() {
  if constexpr (std::is_same_v<T, bool>) return "bool";
  if constexpr (std::is_same_v<T, uint8_t>) return "uint8_t";
  if constexpr (std::is_same_v<T, int8_t>) return "int8_t";
  if constexpr (std::is_same_v<T, uint16_t>) return "uint16_t";
  if constexpr (std::is_same_v<T, int16_t>) return "int16_t";
  if constexpr (std::is_same_v<T, uint32_t>) return "uint32_t";
  if constexpr (std::is_same_v<T, int32_t>) return "int32_t";
  if constexpr (std::is_same_v<T, uint64_t>) return "uint64_t";
  if constexpr (std::is_same_v<T, int64_t>) return "int64_t";
  if constexpr (std::is_same_v<T, float>) return "float";
  if constexpr (std::is_same_v<T, double>) return "double";
  if constexpr (std::is_array_v<T>) {
    using Elem = std::remove_extent_t<T>;
    return cpp_type_name_str<Elem>() + "[" + std::to_string(std::extent_v<T>) + "]";
  }
  if constexpr (std::meta::has_identifier(^^T)) {
    return std::string(std::meta::identifier_of(^^T));
  }
  return std::string(std::meta::display_string_of(^^T));
}

template <typename T>
struct is_std_array : std::false_type {};

template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

template <typename T>
std::string field_type_name() {
  if constexpr (is_std_array<T>::value) {
    using Elem = typename T::value_type;
    return "std::array<" + cpp_type_name_str<Elem>() + ", " + std::to_string(std::tuple_size_v<T>) + ">";
  }
  return cpp_type_name_str<T>();
}

template <std::meta::info Type>
consteval std::string_view py_hint() {
  using T = typename[:Type:];
  if constexpr (std::is_same_v<T, bool>) return "bool"sv;
  if constexpr (std::is_integral_v<T>) return "int"sv;
  if constexpr (std::is_floating_point_v<T>) return "float"sv;
  if constexpr (is_std_array<T>::value) return "list[float]"sv;
  return "Any"sv;
}

template <MsgId Target, MsgId... Ids>
consteval bool contains_msg(ipc::MsgList<Ids...>) {
  return ((Target == Ids) || ...);
}

template <typename Component, MsgId Id>
consteval bool component_subscribes() {
  return contains_msg<Id>(typename Component::Subscribes{});
}

template <typename Component, MsgId Id>
consteval bool component_publishes() {
  return contains_msg<Id>(typename Component::Publishes{});
}

using Components = std::tuple<balancer_doc::ImuService,
                              balancer_doc::ControlService,
                              balancer_doc::MotorService,
                              ipc::UdpBridge>;
using BalancerMessages = ipc::MsgList<ipc::ImuData, ipc::JoystickCommand, ipc::MotorTargets, ipc::SystemTelemetry>;

template <typename Tuple, MsgId Id, std::size_t... Is>
void print_publishers_impl(std::index_sequence<Is...>) {
  bool first = true;
  (..., [&] {
    using Comp = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_publishes<Comp, Id>()) {
      if (!first) std::cout << ", ";
      std::cout << "`" << cpp_type_name_str<Comp>() << "`";
      first = false;
    }
  }());
  if (first) std::cout << "_None_";
}

template <typename Tuple, MsgId Id>
void print_publishers() {
  print_publishers_impl<Tuple, Id>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
}

template <typename Tuple, MsgId Id, std::size_t... Is>
void print_subscribers_impl(std::index_sequence<Is...>) {
  bool first = true;
  (..., [&] {
    using Comp = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_subscribes<Comp, Id>()) {
      if (!first) std::cout << ", ";
      std::cout << "`" << cpp_type_name_str<Comp>() << "`";
      first = false;
    }
  }());
  if (first) std::cout << "_None_";
}

template <typename Tuple, MsgId Id>
void print_subscribers() {
  print_subscribers_impl<Tuple, Id>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
}

template <typename T>
void emit_field_table() {
  constexpr std::size_t N = get_fields_size<T>();
  std::cout << "| Field | C++ Type | Python Type | Bytes | Offset |\n";
  std::cout << "|---|---|---|---:|---:|\n";

  [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    (..., [&] {
      constexpr auto field = StructArrHolder<T, N>::arr[Is];
      constexpr auto type = std::meta::type_of(field);
      using FieldT = typename[:type:];
      std::cout << "| `" << std::meta::identifier_of(field) << "`"
                << " | `" << field_type_name<FieldT>() << "`"
                << " | `" << py_hint<type>() << "`"
                << " | " << sizeof(FieldT)
                << " | " << std::meta::offset_of(field).bytes
                << " |\n";
    }());
  }(std::make_index_sequence<N>{});
  std::cout << "\n";
}

template <typename Payload, MsgId Id>
void emit_payload_doc() {
  constexpr auto payload_ref = ^^Payload;
  constexpr auto desc = get_desc<payload_ref>();

  std::cout << "### `MsgId::" << MessageTraits<Id>::name << "`\n\n";
  if (desc.text[0] != '\0') {
    std::cout << "> " << desc.text << "\n\n";
  }
  std::cout << "- Numeric ID: `" << static_cast<uint16_t>(Id) << "`\n";
  std::cout << "- Payload type: `" << cpp_type_name_str<Payload>() << "`\n";
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
    using Comp = std::tuple_element_t<Is, Tuple>;
    constexpr auto ref = ^^Comp;
    constexpr auto desc = get_desc<ref>();
    std::cout << "### `" << cpp_type_name_str<Comp>() << "`\n\n";
    if (desc.text[0] != '\0') {
      std::cout << "> " << desc.text << "\n\n";
    }
    std::cout << "- Publishes: ";
    []<MsgId... Ids>(ipc::MsgList<Ids...>) {
      bool first = true;
      (((std::cout << (first ? "" : ", ") << "`" << MessageTraits<Ids>::name << "`"), first = false), ...);
      if (first) std::cout << "_None_";
    }(typename Comp::Publishes{});
    std::cout << "\n";
    std::cout << "- Subscribes: ";
    []<MsgId... Ids>(ipc::MsgList<Ids...>) {
      bool first = true;
      (((std::cout << (first ? "" : ", ") << "`" << MessageTraits<Ids>::name << "`"), first = false), ...);
      if (first) std::cout << "_None_";
    }(typename Comp::Subscribes{});
    std::cout << "\n\n";
  }());
}

void emit_components() {
  emit_components_impl<Components>(std::make_index_sequence<std::tuple_size_v<Components>>{});
}

template <MsgId... Ids>
void emit_payloads(ipc::MsgList<Ids...>) {
  (emit_payload_doc<typename MessageTraits<Ids>::Payload, Ids>(), ...);
}

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
    if (std::isalnum(uc)) {
      out.push_back(ch);
    } else {
      out.push_back('_');
    }
  }
  if (out.empty() || std::isdigit(static_cast<unsigned char>(out[0]))) {
    out.insert(out.begin(), '_');
  }
  return out;
}

inline std::string first_sentence(std::string_view s) {
  auto dot = s.find('.');
  if (dot == std::string_view::npos) return std::string(s);
  return std::string(s.substr(0, dot + 1));
}

inline std::string wrap_words(std::string_view s, std::size_t width = 40) {
  std::string out;
  std::string word;
  std::size_t col = 0;
  auto flush_word = [&] {
    if (word.empty()) return;
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

using EdgeMap = std::map<std::pair<std::string, std::string>, std::set<std::string>>;

template <typename Tuple, MsgId Id, std::size_t... Is>
void collect_inbound_edges_impl(EdgeMap& edges, std::index_sequence<Is...>, std::string_view mname) {
  const std::string ub = dot_id("UdpBridge");
  (..., [&] {
    using Comp = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_subscribes<Comp, Id>() && !std::is_same_v<Comp, ipc::UdpBridge>) {
      edges[{ub, dot_id(cpp_type_name_str<Comp>())}].insert(std::string(mname));
    }
  }());
}

template <typename Tuple, MsgId Id>
void collect_inbound_edges(EdgeMap& edges, std::string_view mname) {
  collect_inbound_edges_impl<Tuple, Id>(edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{}, mname);
}

template <typename Tuple, MsgId Id, std::size_t... Is>
void collect_outbound_edges_impl(EdgeMap& edges, std::index_sequence<Is...>, std::string_view mname) {
  const std::string ub = dot_id("UdpBridge");
  (..., [&] {
    using Comp = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_publishes<Comp, Id>() && !std::is_same_v<Comp, ipc::UdpBridge>) {
      edges[{dot_id(cpp_type_name_str<Comp>()), ub}].insert(std::string(mname));
    }
  }());
}

template <typename Tuple, MsgId Id>
void collect_outbound_edges(EdgeMap& edges, std::string_view mname) {
  collect_outbound_edges_impl<Tuple, Id>(edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{}, mname);
}

template <typename Tuple, MsgId Id, std::size_t... Is>
void collect_internal_edges_impl(EdgeMap& edges,
                                 std::index_sequence<Is...>,
                                 std::string_view mname,
                                 std::string_view pub_name) {
  (..., [&] {
    using Comp = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_subscribes<Comp, Id>() && !std::is_same_v<Comp, ipc::UdpBridge>) {
      edges[{dot_id(pub_name), dot_id(cpp_type_name_str<Comp>())}].insert(std::string(mname));
    }
  }());
}

template <typename Tuple, MsgId Id, std::size_t... Is>
void collect_internal_edges_pub_impl(EdgeMap& edges, std::index_sequence<Is...>, std::string_view mname) {
  (..., [&] {
    using Comp = std::tuple_element_t<Is, Tuple>;
    if constexpr (component_publishes<Comp, Id>() && !std::is_same_v<Comp, ipc::UdpBridge>) {
      collect_internal_edges_impl<Tuple, Id>(
          edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{}, mname, cpp_type_name_str<Comp>());
    }
  }());
}

template <typename Tuple, MsgId Id>
void collect_internal_edges(EdgeMap& edges, std::string_view mname) {
  collect_internal_edges_pub_impl<Tuple, Id>(edges, std::make_index_sequence<std::tuple_size_v<Tuple>>{}, mname);
}

template <typename ComponentsT, MsgId Id>
void collect_msg_edges(EdgeMap& edges, std::set<std::string>& inbound_msg_names, std::set<std::string>& outbound_msg_names) {
  constexpr bool cxx_sub = []<std::size_t... Is>(std::index_sequence<Is...>) {
    return ((component_subscribes<std::tuple_element_t<Is, ComponentsT>, Id>() &&
             !std::is_same_v<std::tuple_element_t<Is, ComponentsT>, ipc::UdpBridge>) || ...);
  }(std::make_index_sequence<std::tuple_size_v<ComponentsT>>{});
  constexpr bool cxx_pub = []<std::size_t... Is>(std::index_sequence<Is...>) {
    return ((component_publishes<std::tuple_element_t<Is, ComponentsT>, Id>() &&
             !std::is_same_v<std::tuple_element_t<Is, ComponentsT>, ipc::UdpBridge>) || ...);
  }(std::make_index_sequence<std::tuple_size_v<ComponentsT>>{});
  constexpr bool py_sub = component_subscribes<ipc::UdpBridge, Id>();
  constexpr bool py_pub = component_publishes<ipc::UdpBridge, Id>();
  constexpr std::string_view mname = MessageTraits<Id>::name;

  if (py_pub && cxx_sub && !cxx_pub && !py_sub) {
    inbound_msg_names.insert(std::string(mname));
    collect_inbound_edges<ComponentsT, Id>(edges, mname);
  } else if (cxx_pub && py_sub && !py_pub && !cxx_sub) {
    outbound_msg_names.insert(std::string(mname));
    collect_outbound_edges<ComponentsT, Id>(edges, mname);
  } else if ((py_pub || py_sub) && (cxx_pub || cxx_sub)) {
    inbound_msg_names.insert(std::string(mname));
    collect_inbound_edges<ComponentsT, Id>(edges, mname);
    outbound_msg_names.insert(std::string(mname));
    collect_outbound_edges<ComponentsT, Id>(edges, mname);
  } else if (!py_pub && !py_sub && cxx_pub && cxx_sub) {
    collect_internal_edges<ComponentsT, Id>(edges, mname);
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
  os << "  node [fontname=\"Helvetica,Arial,sans-serif\", shape=rect, style=\"filled,rounded\", fixedsize=false, margin=\"0.12,0.08\", fontsize=28, fontcolor=\"#FFFFFF\", penwidth=4, color=\"#F1F5F9\"];\n\n";
  os << "  edge [fontname=\"Helvetica,Arial,sans-serif\", fontsize=22, fontcolor=\"#CBD5E1\", color=\"#64748B\", penwidth=3.0, arrowsize=1.4, labelfloat=false, labeldistance=0.8, labelangle=10];\n\n";

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
      using Comp = std::tuple_element_t<Is, Components>;
      constexpr auto ref = ^^Comp;
      const std::string cname = cpp_type_name_str<Comp>();
      std::string desc_text = wrap_words(first_sentence(get_desc<ref>().text), 38);
      if (desc_text.empty()) desc_text = cname;
      const bool is_udp = std::is_same_v<Comp, ipc::UdpBridge>;
      os << "    " << dot_id(cname) << " [\n";
      os << "      fillcolor=\"" << (is_udp ? "#0F766E" : "#0369A1") << "\",\n";
      os << "      label=<<B><FONT POINT-SIZE=\"" << (is_udp ? "38" : "30") << "\">"
         << html_escape(cname) << "</FONT></B><BR/><FONT POINT-SIZE=\"24\">"
         << html_escape(desc_text) << "</FONT>>\n";
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
  os << "    TX [fillcolor=\"#B45309\", label=<<B><FONT POINT-SIZE=\"30\">TX Socket</FONT></B><BR/><FONT POINT-SIZE=\"24\">Port 9000 (Inbound)</FONT>>];\n";
  os << "    RX [fillcolor=\"#B45309\", label=<<B><FONT POINT-SIZE=\"30\">RX Socket</FONT></B><BR/><FONT POINT-SIZE=\"24\">Client Ephemeral Port (Outbound)</FONT>>];\n";
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
  os << "    TestCase [fillcolor=\"#047857\", label=<<B><FONT POINT-SIZE=\"34\">Test Case / Fixtures</FONT></B>>];\n";
  os << "  }\n\n";

  EdgeMap edge_map;
  std::set<std::string> inbound_msg_names;
  std::set<std::string> outbound_msg_names;
  collect_msg_edges<Components, ipc::ImuData>(edge_map, inbound_msg_names, outbound_msg_names);
  collect_msg_edges<Components, ipc::JoystickCommand>(edge_map, inbound_msg_names, outbound_msg_names);
  collect_msg_edges<Components, ipc::MotorTargets>(edge_map, inbound_msg_names, outbound_msg_names);
  collect_msg_edges<Components, ipc::SystemTelemetry>(edge_map, inbound_msg_names, outbound_msg_names);

  os << "  TestCase -> TX [label=\"send_msg\", color=\"#F1F5F9\", penwidth=5];\n";
  os << "  TX -> " << dot_id("UdpBridge") << " [label=\"Inbound Traffic:\\n"
     << dot_escape_label(build_label(inbound_msg_names)) << "\", color=\"#F1F5F9\", penwidth=5, fontsize=25];\n\n";
  os << "  // Internal Service Logic\n";
  for (const auto& [edge, names] : edge_map) {
    os << "  " << edge.first << " -> " << edge.second
       << " [label=\"" << dot_escape_label(build_label(names)) << "\", style=dashed];\n";
  }
  os << "\n  RX -> " << dot_id("UdpBridge") << " [label=\"Outbound Traffic:\\n"
     << dot_escape_label(build_label(outbound_msg_names))
     << "\", color=\"#F1F5F9\", penwidth=5, dir=back, fontsize=25, arrowtail=normal];\n";
  os << "  TestCase -> RX [label=\"recv_msg\", color=\"#F1F5F9\", penwidth=5, dir=back, arrowtail=normal];\n";
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

  std::cout << "# balancer_bot IPC Reference\n\n";
  std::cout << "> Auto-generated by `generate_balancer_docs` using GCC trunk reflection.\n\n";

  std::cout << "## Overview\n\n";
  std::cout << "This document describes the current balancer SIL/IPC surface: the UDP bridge, the three service wrappers, and the fixed-size payload structs exchanged across the message bus.\n\n";
  std::cout << "The current runtime flow is straightforward:\n\n";
  std::cout << "1. Python tests inject `JoystickCommand` and `ImuData` through `UdpBridge`.\n";
  std::cout << "2. `ControlService` consumes those inputs and publishes `MotorTargets` plus `SystemTelemetry`.\n";
  std::cout << "3. `MotorService` consumes `MotorTargets`, and `UdpBridge` relays outbound traffic back to Python.\n\n";

  std::cout << "![IPC Flow Diagram](ipc_flow.svg)\n\n";

  std::cout << "## Components\n\n";
  emit_components();

  std::cout << "## Message Payloads\n\n";
  emit_payloads(BalancerMessages{});

  std::cout << "## Regenerating\n\n";
  std::cout << "```bash\n";
  std::cout << "cmake -S . -B build\n";
  std::cout << "cmake --build build --target balancer_docs\n";
  std::cout << "```\n";

  return 0;
}
