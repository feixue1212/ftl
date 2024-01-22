#ifndef __FTL_FAST__
#define __FTL_FAST__

#include <cinttypes>
#include <unordered_map>
#include <vector>

#include "ftl/abstract_ftl.hh"
#include "ftl/common/block.hh"
#include "ftl/ftl.hh"
#include "pal/pal.hh"

namespace SimpleSSD {

namespace FTL {

class FAST : public AbstractFTL {
  private:
    PAL::PAL *pPAL;

    ConfigReader &conf;

    struct SW_Block_Info {
      uint32_t block_idx;
      uint32_t belong_data_block_physical_idx;
      uint32_t belong_data_block_logical_idx;
      uint32_t cur_page_idx;
      uint32_t res_page_cnt;
      uint32_t last_access_ppi;
    };
    SW_Block_Info* sw_info;
    std::unordered_map<uint32_t, uint32_t> block_table;//存物理块到逻辑块的
    std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> page_table;//存逻辑页到物理页的
    std::unordered_map<uint32_t, Block> blocks;
    std::list<std::pair<uint32_t, Block>> freeBlocks;
    std::vector<std::pair<uint32_t, Block*>> RWBlocks;


    bool bRandomTweak;
    //作废一个RW块的页面，在write时所需
    bool invalidRWPage(uint64_t, uint32_t&, uint32_t&);
    //计算物理页面号：
    inline uint64_t calculatePageNumber(uint32_t, uint32_t);
    //计算有效页和无效页的总数：
    void calculateTotalPages(uint64_t&, uint64_t&);
    //分配一个新的空闲块：
    //完全替换：
    void SwitchMerge(uint64_t&);
    //部分合并：
    void PartialMerge(uint64_t&);
    void blockErase(uint32_t, uint64_t&);

    std::pair<uint32_t, uint32_t> FAST::GarbageCollection(uint64_t&);
    std::pair<uint32_t, uint32_t> getRWPage(uint64_t&);

    uint32_t getFreeBlock();
    void ReadPage(uint32_t, uint32_t, Request&, uint64_t&);
    void WritePage(uint32_t, uint32_t, uint32_t, uint32_t, Request&, bool, bool, uint64_t&);

    void configIOFlag(PAL::Request&);
    void ReadThenWrite(uint32_t, Block*, uint32_t, Block*, uint32_t, uint32_t, uint64_t&);
    //读
    void readInternal(Request&, uint64_t&);
    //写
    void writeInternal(Request&, uint64_t&, bool = true);
  public:
    FAST(ConfigReader &, Parameter &, PAL::PAL *, DRAM::AbstractDRAM *);
    ~FAST();

    bool initialize() override;

    void read(Request &, uint64_t &) override;
    void write(Request &, uint64_t &) override;
    //无需实现：
    void trim(Request &, uint64_t &) override;
    void format(LPNRange &, uint64_t &) override;
    Status *getStatus(uint64_t, uint64_t) override;
};

}

}

#endif