#pragma once

#include <map>
#include <Import.h>
#include <BusAccessor.h>
#include <BasicsBusDef.h>
#include <SensorBusDef.h>


struct UserData
{
    unsigned int time;
    unsigned int busId;
    void* state;
    std::string name;
    std::string outputPath;
    std::map<std::string, std::string> parameters;
};


extern "C" void PANOSIM_API ModelStart(UserData*);

extern "C" void PANOSIM_API ModelOutput(UserData*);

extern "C" void PANOSIM_API ModelTerminate(UserData*);


class BusAccessor
{
public:
    explicit BusAccessor(int busId, std::string_view key, std::string_view format);
    virtual ~BusAccessor();
    void* GetHeader() const;
    void* GetBody() const;
};

class DoubleBusReader
{
public:
    explicit DoubleBusReader(int busId, std::string_view key, std::string_view format);
    virtual ~DoubleBusReader();
    CBusAccessor* GetReader(std::uint32_t nCurrentTime) const;
};
