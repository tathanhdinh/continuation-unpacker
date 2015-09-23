#include "../type/trace.pb.h"
#include "../type/instruction.h"

#include "../tinyformat.h"

#include <memory>
#include <fstream>
#include <locale>

p_instructions_t trace = p_instructions_t{};
map_address_instruction_t cached_ins_at_addr = map_address_instruction_t();

static std::ifstream protobuf_trace_file;

static auto parse_trace_header () -> void
{
  auto header_size = int{0};
  protobuf_trace_file.read(reinterpret_cast<char*>(&header_size), sizeof(decltype(header_size)));
  if (!protobuf_trace_file) throw std::range_error("reading header size error");

  auto header_buffer = std::shared_ptr<char>(new char[header_size], std::default_delete<char[]>());
  protobuf_trace_file.read(header_buffer.get(), header_size);
  if (!protobuf_trace_file) throw std::range_error("reading header error");

  return;
}


#define REG_READ true
#define REG_WRITE false
template<bool read_or_write>
static auto get_register_info (p_instruction_t ins, const trace_format::ins_con_info_t& pb_con_info) -> void
{
  const auto& pb_reg_info = read_or_write ? pb_con_info.read_register() : pb_con_info.write_register();
  auto& ins_regs = read_or_write ? ins->read_register : ins->written_register;

  auto reg_name = pb_reg_info.name();

  auto pb_reg_value = pb_reg_info.value();
  auto reg_value = uint32_t{0};

  switch (pb_reg_value.typeid_()) {
  case trace_format::BIT8:
    reg_value = pb_reg_value.value_8();
    break;

  case trace_format::BIT16:
    reg_value = pb_reg_value.value_16();
    break;

  case trace_format::BIT32:
    reg_value = pb_reg_value.value_32();
    break;

  default:
    assert(false);
    break;
  }

  ins_regs[reg_name] = reg_value;

//  std::transform(reg_name.begin(), reg_name.end(), reg_name.begin(), ::toupper);

//  for (auto& reg_enum_val : ins_regs) {
//    auto reg_enum = std::get<0>(reg_enum_val);
//    if (reg_name == xed_reg_enum_t2str(reg_enum)) {
//      auto pb_value = pb_reg_info.value();

//      auto reg_size = xed_get_register_width_bits(reg_enum);
//      auto reg_val = uint32_t{0};

//      switch (reg_size) {
//        case 8:
//          assert(pb_value.typeid_() == trace_format::BIT8);
//          reg_val = pb_value.value_8();
//          break;

//        case 16:
//          assert(pb_value.typeid_() == trace_format::BIT16);
//          reg_val = pb_value.value_16();
//          break;

//        case 32:
//          assert(pb_value.typeid_() == trace_format::BIT32);
//          reg_val = pb_value.value_32();
//          break;

//        default:
//          assert(false);
//          break;
//      }

//      ins->read_registers[reg_enum] = reg_val;
//      break;
//    }
//  }

  return;
}


#define MEM_LOAD true
#define MEM_STORE false
template<bool load_or_store>
static auto get_memory_info (p_instruction_t ins, const trace_format::ins_con_info_t& pb_con_info) -> void
{
  const auto& pb_mem_info = load_or_store ? pb_con_info.load_memory() : pb_con_info.store_memory();
  auto& ins_mem = load_or_store ? ins->load_memory : ins->store_memmory;

  const auto& pb_mem_addr = pb_mem_info.address();
  assert(pb_mem_addr.has_value_32());
  auto mem_addr = pb_mem_addr.value_32();

  const auto& pb_mem_val = pb_mem_info.value();
  auto mem_val = uint32_t{0};

  switch (pb_mem_val.typeid_()) {
    case trace_format::BIT8:
      mem_val = pb_mem_val.value_8();
      break;

    case trace_format::BIT16:
      mem_val = pb_mem_val.value_16();
      break;

    case trace_format::BIT32:
      mem_val = pb_mem_val.value_32();
      break;

    default:
      assert(false);
      break;
  }

  ins_mem[mem_addr] = mem_val;

  return;
}

static auto parse_chunk_from_buffer (const char* buffer, int buffer_size) -> void
{
//  auto inst_chunk = trace_format::chunk_t();
  static auto inst_chunk = trace_format::chunk_t();
  inst_chunk.Clear();

  if (!inst_chunk.ParseFromArray(buffer, buffer_size)) throw std::domain_error("parsing chunk error");

//  tfm::printfln("number of instructions in chunk: %d", inst_chunk.body_size());
  auto body_num = inst_chunk.body_size();
  for (auto i = 0; i < body_num; ++i) {
    const auto& body = inst_chunk.body(i);

    if (body.typeid_() == trace_format::INSTRUCTION) {
      const auto& pb_inst_info = body.instruction();

      const auto& pb_inst_addr = pb_inst_info.address();
      auto ins_addr = pb_inst_addr.value_32();

      if (cached_ins_at_addr.find(ins_addr) == std::end(cached_ins_at_addr)) {
        const auto& pb_inst_opcode = pb_inst_info.opcode();
        auto opcode_size = pb_inst_opcode.size();
        auto opcode_buffer = pb_inst_opcode.data();

        cached_ins_at_addr[ins_addr] = std::make_shared<instruction>(ins_addr, opcode_buffer, opcode_size);
      }

      auto curr_ins = std::make_shared<instruction>(*cached_ins_at_addr[ins_addr]);

      auto con_info_num = pb_inst_info.concrete_info_size();
      for (auto info_idx = decltype(con_info_num){0}; info_idx < con_info_num; ++info_idx) {
        const auto& pb_con_info = pb_inst_info.concrete_info(info_idx);

        switch (pb_con_info.typeid_()) {
          case trace_format::REGREAD:
            get_register_info<REG_READ>(curr_ins, pb_con_info);
            break;

          case trace_format::REGWRITE:
            get_register_info<REG_WRITE>(curr_ins, pb_con_info);
            break;

          case trace_format::MEMLOAD:
            get_memory_info<MEM_LOAD>(curr_ins, pb_con_info);
            break;

          case trace_format::MEMSTORE:
            get_memory_info<MEM_STORE>(curr_ins, pb_con_info);
            break;

        default:
          break;
        }
      }

      trace.push_back(curr_ins);

//      const auto& con_info_num = pb_inst_info.concrete_info_size();
//      for (auto idx = 0; idx < con_info_num; ++idx) {
//        const auto& con_info_idx = pb_inst_info.concrete_info(idx);
//        if (con_info_idx.typeid_() == trace_format::NEXT_ADDRESS) {
//          auto pb_next_addr = con_info_idx.next_address();
//          auto next_addr = pb_next_addr.value_32();
//          tfm::printfln("next address 0x%x", next_addr);
//        }
//      }
    }
  }

//  inst_chunk.Clear();

  return;
}


static auto parse_trace_chunks () -> void
{
  auto chunk_size = int{0};

  while (true) {
    protobuf_trace_file.read(reinterpret_cast<char*>(&chunk_size), sizeof(decltype(chunk_size)));
    if (!protobuf_trace_file) throw std::range_error("reading chunk size error");

    auto chunk_buffer = std::shared_ptr<char>(new char[chunk_size], std::default_delete<char[]>());
    protobuf_trace_file.read(chunk_buffer.get(), chunk_size);

//    tfm::printfln("chunk size: %d bytes", chunk_size);
    parse_chunk_from_buffer(chunk_buffer.get(), chunk_size);
  }

  return;
}


/* ===================================== exported functions ===================================== */

auto print_instructions_parsed_from_file (const std::string& filename) -> void
{
  try {
//    protobuf_trace_file = std::ifstream(filename.c_str(), std::ifstream::in | std::ifstream::binary);
    protobuf_trace_file.open(filename.c_str(), std::ifstream::in | std::ifstream::binary);

    xed_tables_init();

    parse_trace_header();
    parse_trace_chunks();

    for (const auto& addr_inst : cached_ins_at_addr) {
      tfm::printfln("0x%x %s", std::get<0>(addr_inst), std::get<1>(addr_inst)->disassemble);
    }
  }
  catch (const std::exception& expt) {
    tfm::printfln("%s", expt.what());
  }

  google::protobuf::ShutdownProtobufLibrary();
  protobuf_trace_file.close();

  return;
}

auto parse_instructions_from_file (const std::string& filename) -> const p_instructions_t&
{
  trace.clear();

  try {
    tfm::printfln("===== reading protobuf trace (input file: %s)...", filename);
//    protobuf_trace_file = std::move(std::ifstream(filename.c_str(), std::ifstream::in | std::ifstream::binary));
    protobuf_trace_file.open(filename.c_str(), std::ifstream::in | std::ifstream::binary);
    if (!protobuf_trace_file) throw std::runtime_error("cannot open file to read");

    xed_tables_init();

    parse_trace_header();
    parse_trace_chunks();    
  }
  catch (const std::exception& expt) {
    tfm::printfln("%s : %s instruction parsed", expt.what(), trace.size());
  }

  protobuf_trace_file.close();
  google::protobuf::ShutdownProtobufLibrary();
  return trace;
}


template<bool read_or_write>
static auto save_register_info (p_instruction_t ins, std::ofstream& trace_file) -> void
{
  auto ins_regs = read_or_write ? ins->read_register : ins->written_register;

  if (ins_regs.size() > 0) {
//    const auto& rw_str_info = read_or_write ? " RR: " : " RW: ";

//    tfm::format(trace_file, rw_str_info);
    for (const auto& reg_name_value : ins_regs) {
//      auto reg_name = std::string(xed_reg_enum_t2str(std::get<0>(reg_enum_val)));
//      std::transform(std::begin(reg_name), std::end(reg_name), std::begin(reg_name), ::tolower);

      // data segment is used in real mode only
      if (std::get<0>(reg_name_value) != "ds") {
//        auto reg_value = std::get<1>(reg_name_value);
//        auto format_str = (reg_value <= 127) && std::isprint(reg_value) ? "[%s:%c(%c)] " : "[%s:0x%x(%c)] ";

//        tfm::format(trace_file, format_str,
//                    std::get<0>(reg_name_value), reg_value, read_or_write ? 'r' : 'w');

        tfm::format(trace_file, "[%s:0x%x(%c)] ",
                    std::get<0>(reg_name_value), std::get<1>(reg_name_value), read_or_write ? 'r' : 'w');
      }
    }
  }
  return;
}


template<bool load_or_store>
static auto save_memory_info (p_instruction_t ins, std::ofstream& trace_file) -> void
{
  auto ins_mem = load_or_store ? ins->load_memory : ins->store_memmory;

  if (ins_mem.size() > 0) {
//    const auto& ls_str_info = load_or_store ? " MR: " : " MW: ";

//    tfm::format(trace_file, ls_str_info);
    for (const auto& mem_addr_val : ins_mem) {
//      auto mem_value = std::get<1>(mem_addr_val);
//      auto format_str = (mem_value <= 127) && std::isprint(mem_value) ? "[0x%x:%c(%c)] " : "[0x%x:0x%x(%c)] ";

//      tfm::format(trace_file, format_str,
//                  std::get<0>(mem_addr_val), mem_value, load_or_store ? 'r' : 'w');

      tfm::format(trace_file, "[0x%x:0x%x(%c)] ",
                  std::get<0>(mem_addr_val), std::get<1>(mem_addr_val), load_or_store ? 'r' : 'w');
    }
  }
  return;
}

auto save_trace_to_file (const std::string& filename) -> void
{
  try {
    std::ofstream trace_file(filename.c_str(), std::ofstream::trunc);
    if (!trace_file) throw std::runtime_error("cannot open file to write");

    for (const auto& inst : trace) {
      tfm::format(trace_file, "0x%x  %-40s", inst->address, inst->disassemble);

//      save_register_info<REG_READ>(inst, trace_file);
//      save_register_info<REG_WRITE>(inst, trace_file);
//      save_memory_info<MEM_LOAD>(inst, trace_file);
//      save_memory_info<MEM_STORE>(inst, trace_file);

      tfm::format(trace_file, "\n");
    }

    trace_file.close();
    tfm::printfln("output trace file: %s", filename);
  }
  catch (const std::exception& expt) {
    tfm::printfln("%s", expt.what());
  }

  return;
}

extern auto split_trace_into_chunks (const p_instructions_t& trace) -> std::vector<p_instructions_t>;
extern auto split_trace_into_chunks (const p_instructions_t& trace, uint32_t start_addr) -> std::vector<p_instructions_t>;

auto save_chunks_to_file (const std::string& filename) -> void
{
  try {
    auto ins_chunks = split_trace_into_chunks(trace);
//    auto ins_chunks = split_trace_into_chunks(trace, 0x8048709);

    auto chunk_idx = uint32_t{0};
    for (const auto& chunk : ins_chunks) {
      auto chunk_idx_str = std::to_string(chunk_idx);
      auto chunk_filename = filename + chunk_idx_str;

      std::ofstream output_file(chunk_filename.c_str(), std::ofstream::trunc);
      if (!output_file) throw std::logic_error("cannot open output file");

      for (const auto& inst : chunk) {
        tfm::format(output_file, "0x%x  %-40s", inst->address, inst->disassemble);
//        tfm::format(output_file, "%-40s", inst->disassemble);

        save_register_info<REG_READ>(inst, output_file);
        save_register_info<REG_WRITE>(inst, output_file);
        save_memory_info<MEM_LOAD>(inst, output_file);
        save_memory_info<MEM_STORE>(inst, output_file);

        tfm::format(output_file, "\n");
      }

      output_file.close();
      tfm::printfln("output chunk file: %s", chunk_filename);

      ++chunk_idx;
    }
  }
  catch (const std::exception& expt) {
    tfm::printfln("%s", expt.what());
  }

  return;
}
