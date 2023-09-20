#pragma once

#include <stdlib.h>
#include <string.h>
#include <memory>

#define MBX_DATABYTES_MAX 128

class CDataBuffer
{
private:
	size_t _size;
	size_t _maxSize;
	void* _outerPtr;
	char* _byteArray;
	bool _hasBadAlloc;

	bool ExpandSize(size_t reqSize)
	{
		// New array size is twice the original size
		// If reqSize is greater than 2*_max, multiply by two again
		size_t newMaxSize = _maxSize;
		do {
			newMaxSize *= 2;
		} while (newMaxSize < reqSize);

		// allocate new memory
		char* newArray = (char*)malloc(newMaxSize);
		if (newArray == NULL)
		{
			_hasBadAlloc = true;
			return false;
		}
		memset(newArray, 0, newMaxSize);

		// keep old address to free the memory later
		char* temp = _byteArray;

		// copy data from old memory
		memcpy(newArray, _byteArray, _maxSize);

		// assign new size and address
		_byteArray = newArray;
		_maxSize = newMaxSize;

		free(temp);
		return true;
	}

public:
	CDataBuffer(size_t max)
		: _size(0), _maxSize(max), _outerPtr(NULL)
		, _hasBadAlloc(false)
		
	{
		_byteArray = (char*)malloc(max);
		if (_byteArray == NULL)
		{
			_maxSize = 0;
			_hasBadAlloc = true;
			return;
		}
		memset(_byteArray, 0, max);
	}
	CDataBuffer(void* ptr, size_t max) // use outer data source
		: _size(0), _maxSize(max), _outerPtr(ptr)
		, _byteArray(NULL), _hasBadAlloc(false)
	{}
	virtual ~CDataBuffer()
	{
		free(_byteArray);
	}

	void Clear()
	{
		Resize(0);
		_hasBadAlloc = false;
	}

	size_t GetSize()
	{
		return _size;
	}
	size_t GetMaxSize()
	{
		return _maxSize;
	}

	bool SetSize(size_t size)
	{
		if (size > _maxSize)
			return false;

		_size = size;
		return true;
	}
	bool Resize(size_t newSize)
	{
		if (_outerPtr != NULL)
			return false;

		_size = newSize;

		if (newSize > _maxSize)
			return ExpandSize(newSize);

		return true;
	}
	bool HasBadAlloc()
	{
		return _hasBadAlloc;
	}

	char* GetPointer()
	{
		return _outerPtr == NULL ?
			_byteArray :
			(char*)_outerPtr;
	}
};

class CArchive
{
private:
	std::unique_ptr<CDataBuffer> _dataBufferPtr;
	int _index;
	bool _hasNullPtr;

	bool Resize(size_t newSize)
	{
		return _dataBufferPtr->Resize(newSize);
	}

public:
	CArchive(size_t maxSize = MBX_DATABYTES_MAX)
		: _dataBufferPtr(new CDataBuffer(maxSize))
		, _index(0), _hasNullPtr(false)
	{}
	virtual ~CArchive(void) {}

	void Clear()
	{
		_index = 0;
		_hasNullPtr = false;
		_dataBufferPtr->Clear();
	}

	size_t GetSize()
	{
		return _dataBufferPtr->GetSize();
	}
	size_t GetMaxSize()
	{
		return _dataBufferPtr->GetMaxSize();
	}
	char* GetData()
	{
		return _dataBufferPtr->GetPointer();
	}

	bool HasError_NullPtr()
	{
		return _hasNullPtr;
	}
	bool HasBadAlloc()
	{
		return _dataBufferPtr->HasBadAlloc();
	}

	// Reset buffer data
	bool Update(size_t size, void* lpData)
	{
		if (Resize(size) == false)
			return false;

		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(bufferPtr, lpData, size);
		return true;
	}
	// Append data after the buffer
	bool Append(size_t size, void* lpData)
	{
		auto bufferSize = GetSize();
		if (Resize(size + bufferSize) == false)
			return false;

		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(bufferPtr + bufferSize, lpData, size);
		return true;
	}

	CArchive& operator << (CDataBuffer& data)
	{
		auto dataPtr = data.GetPointer();
		auto dataSize = data.GetMaxSize(); //.GetSize();
		auto bufferSize = GetSize();
		if (Resize(bufferSize + dataSize) == false)
			return *this;

		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(bufferPtr + bufferSize, dataPtr, dataSize);
		return *this;
	}
	CArchive& operator >> (CDataBuffer& data)
	{
		auto dataPtr = data.GetPointer();
		auto dataSize = data.GetMaxSize(); //.GetSize();
		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(dataPtr, bufferPtr + _index, dataSize);
		_index += dataSize;
		return *this;
	}
	CArchive& operator << (CArchive& data)
	{
		auto dataPtr = data.GetData();
		auto dataSize = data.GetMaxSize();
		auto bufferSize = GetSize();
		if (Resize(bufferSize + dataSize) == false)
			return *this;

		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(bufferPtr + bufferSize, dataPtr, dataSize);
		return *this;
	}
	CArchive& operator >> (CArchive& data)
	{
		auto dataPtr = data.GetData();
		auto dataSize = data.GetMaxSize();
		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(dataPtr, bufferPtr + _index, dataSize);
		_index += dataSize;
		return *this;
	}

  /*
	CArchive& operator << (std::string& data)
	{
		auto dataPtr = (char*)data.c_str();
		auto dataSize = data.length() + 1;
		auto bufferSize = GetSize();
		if (Resize(bufferSize + dataSize) == false)
			return *this;

		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(bufferPtr + bufferSize, dataPtr, dataSize);
		return *this;
	}
	CArchive& operator >> (std::string& data)
	{
		auto bufferPtr = _dataBufferPtr->GetPointer();
		data = (char*)(bufferPtr + _index); //transfer char* -> string

		auto dataSize = data.length() + 1;
		_index += dataSize;
		return *this;
	}
  */

	template<typename T> CArchive& operator << (T& data)
	{
		auto dataPtr = (T*)&data;
		auto dataSize = sizeof(T);
		auto bufferSize = GetSize();
		if (Resize(bufferSize + dataSize) == false)
			return *this;

		auto bufferPtr = _dataBufferPtr->GetPointer();

		// check for NULL
		if (dataPtr == NULL)
		{
			_hasNullPtr = true;
			return *this;
		}

		::memcpy(bufferPtr + bufferSize, dataPtr, dataSize);
		return *this;
	}
	template<typename T> CArchive& operator >> (T& data)
	{
		auto dataPtr = (T*)&data;
		auto dataSize = sizeof(T);
		auto bufferPtr = _dataBufferPtr->GetPointer();
		::memcpy(dataPtr, bufferPtr + _index, dataSize);
		_index += dataSize;
		return *this;
	}
};