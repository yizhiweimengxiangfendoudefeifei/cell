#pragma once

#include <memory>
#include <unordered_map>

class CBusAccessor
{
public:
    CBusAccessor(std::string_view busId, std::string_view key, std::string_view format);
    CBusAccessor(std::string_view busId, std::string_view key, std::string_view format, bool flag);
    virtual ~CBusAccessor();
    using SharedPtr = std::shared_ptr<CBusAccessor>;

    void* GetHeader() const;

    void* GetBody() const;

    const std::string& GetName() const;

protected:
    bool Open(std::string_view busId, std::string_view key, std::string_view format);
    void Close();
    bool ParseFormat(std::string_view format);

    void Init(std::string_view busId, std::string_view key, std::string_view format);

protected:
    bool Valid;
    void* FileMapping;
    mutable void* Memory;
    unsigned long TotalSize;
    unsigned int BodyOffset;
    std::string m_strName;
    bool doubleBusFlag;
};
