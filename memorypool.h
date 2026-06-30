/*
Copyright (c) 2026 Carlos de Diego

This Source Code Form is subject to the terms of the
Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <new>
#include "lib/parallel_hashmap/btree.h"

#define IS_64BIT (UINTPTR_MAX > UINT32_MAX)

namespace mempool {

    // Undefined behaviour when value == 0
    size_t unsafe_bit_scan_forward(const size_t value) {
#if defined(_MSC_VER)
        unsigned long result;
    #if IS_64BIT
        _BitScanForward64(&result, value);
    #else
        _BitScanForward(&result, value);
    #endif
        return result;
#elif defined(__GNUC__) || defined(__clang__)
    #if IS_64BIT
        return __builtin_ctzll(value);
    #else
        return __builtin_ctz(value);
    #endif
#else
    #if IS_64BIT
        const unsigned int tab64[64] = {
             0,  1,  2, 53,  3,  7, 54, 27,
             4, 38, 41,  8, 34, 55, 48, 28,
            62,  5, 39, 46, 44, 42, 22,  9,
            24, 35, 59, 56, 49, 18, 29, 11,
            63, 52,  6, 26, 37, 40, 33, 47,
            61, 45, 43, 21, 23, 58, 17, 10,
            51, 25, 36, 32, 60, 20, 57, 16,
            50, 31, 19, 15, 30, 14, 13, 12
        };
        return tab64[(value & (0 - value)) * 0x022FDD63CC95386D >> 58];
    #else
        static uint8_t tab32[32] = {
             0,  1, 28,  2, 29, 14, 24,  3,
            30, 22, 20, 15, 25, 17,  4,  8,
            31, 27, 13, 23, 21, 19, 16,  7,
            26, 12, 18,  6, 11,  5, 10,  9
        };
        return tab32[(value & (0 - value)) * 0x077CB531U >> 27];
    #endif
#endif
    }

    size_t unsafe_int_log2(size_t value) {
#if defined(_MSC_VER)
        unsigned long result;
    #if IS_64BIT
        _BitScanReverse64(&result, value);
    #else
        _BitScanReverse(&result, value);
    #endif
        return result;
#elif defined(__GNUC__) || defined(__clang__)
    #if IS_64BIT
        return 63 - __builtin_clzll(value);
    #else
        return 31 - __builtin_clz(value);
    #endif
#else
    #if IS_64BIT
        const uint8_t tab64[64] = {
             0, 58,  1, 59, 47, 53,  2, 60,
            39, 48, 27, 54, 33, 42,  3, 61,
            51, 37, 40, 49, 18, 28, 20, 55,
            30, 34, 11, 43, 14, 22,  4, 62,
            57, 46, 52, 38, 26, 32, 41, 50,
            36, 17, 19, 29, 10, 13, 21, 56,
            45, 25, 31, 35, 16,  9, 12, 44,
            24, 15,  8, 23,  7,  6,  5, 63
        };
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        value |= value >> 32;
        return tab64[value * 0x03f6eaf2cd271461 >> 58];
    #else
        const uint8_t tab32[32] = {
             0,  9,  1, 10, 13, 21,  2, 29,
            11, 14, 16, 18, 22, 25,  3, 30,
             8, 12, 20, 28, 15, 17, 24,  7,
            19, 27, 23,  6, 26,  5,  4, 31
        };
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        return tab32[value * 0x07C4ACDDU >> 27];
    #endif
#endif
    }

    struct NoLock
    {
        void lock() {}
        void unlock() {}
    };


    /*************************************************************
     *                  Fixed Size Memory Pool                   *
     ************************************************************/

    // Very simple memory pool with fixed size blocks. allocate() and free() are O(1).
    // Memory overhead is 2 bits per block.
    //
    // For a memory pool with fixed size blocks you only need a stack. The stack is 
    // essentially a list of indexes of free blocks. 
    // - On allocation you pop the top index from it, and use it as your pointer. 
    // - On deallocation, you insert the index at the top.
    // 
    // To reduce memory use, I group blocks into chunks. Each chunk has a bitmap
    // which shows which blocks in it are free. We can quickly find a free block
    // in that bitmap using a bit scan. In this case the stack contains indexes of
    // chunks that are not full. 
    // - On allocation we take the top chunk from the stack, and remove it ONLY if 
    //   it becomes full.
    // - On deallocation we push the chunk into the stack ONLY if it was previously full.

    const int FIXED_POOL_CHUNK_SIZE = sizeof(size_t) * 8;
    
    template<typename LockType = NoLock>
    class FixedMemoryPool
    {
        union {
            uint8_t* buffer = nullptr;  // Buffer containing all blocks and metadata of the pool.
            size_t* nonFullChunks;  // Stack with indexes for chunks that are not yet full. Always at the start of the buffer.
        };
        size_t* chunkBitmaps = nullptr;
        uint8_t* blocks = nullptr;
        
        size_t nonFullChunkCount = 0;
        size_t allocatedBlocks = 0;
        size_t numberBlocks = 0;
        size_t blockSize = 1;

        LockType lock;
        bool allocFallback = false;  // If true and no blocks are available allocate using new[], otherwise throw std::bad_alloc.
        bool externalBuffer = false;  // True if the memory buffers (data, nonFullChunks, chunkBitmaps) come externally

        void release_buffer()
        {
            if (!externalBuffer)
                delete[] buffer;
            buffer = nullptr;
        }

        size_t block_count_from_size(size_t _blockSize, size_t _bufferSize)
        {
            _bufferSize -= 16;
            return size_t(_bufferSize / (_blockSize + 0.25));
        }

        void initialize_internal(uint8_t* _buffer, size_t _blockSize, size_t _numberBlocks, size_t _numberChunks, bool _allocFallback = false)
        {
            release_buffer();
            externalBuffer = false;
            numberBlocks = _numberBlocks;
            blockSize = _blockSize;
            allocatedBlocks = 0;
            nonFullChunkCount = _numberChunks;
            allocFallback = _allocFallback;

            buffer = _buffer;
            nonFullChunks = (size_t*)buffer;
            chunkBitmaps = (size_t*)(buffer + _numberChunks * sizeof(size_t));
            blocks = buffer + _numberChunks * sizeof(size_t) * 2;

            for (size_t i = 0; i < _numberChunks; i++)
                nonFullChunks[i] = i;
            for (size_t i = 0; i < _numberChunks; i++)
                chunkBitmaps[i] = (size_t)-1;
            // If we dont have a multiple of CHUNK_SIZE mark some blocks in last chunk as used
            if (_numberBlocks % FIXED_POOL_CHUNK_SIZE)
                chunkBitmaps[_numberChunks - 1] >>= (FIXED_POOL_CHUNK_SIZE - _numberBlocks % FIXED_POOL_CHUNK_SIZE);
        }

    public:

        FixedMemoryPool() {}
        FixedMemoryPool(size_t _blockSize, size_t _numberBlocks, bool _allocFallback = false)
        {
            restart(_blockSize, _numberBlocks, _allocFallback);
        }
        FixedMemoryPool(size_t _blockSize, uint8_t* _buffer, size_t _bufferSize, bool _allocFallback = false)
        {
            restart(_blockSize, _buffer, _bufferSize, _allocFallback);
        }
        
        ~FixedMemoryPool() 
        {
            release_buffer();
        }

        void restart(size_t _blockSize, size_t _numberBlocks, bool _allocFallback = false)
        {
            size_t numberChunks = (_numberBlocks + FIXED_POOL_CHUNK_SIZE - 1) / FIXED_POOL_CHUNK_SIZE;
            uint8_t* newBuffer = new  uint8_t[_blockSize * _numberBlocks + numberChunks * sizeof(size_t) * 2];
            initialize_internal(newBuffer, _blockSize, _numberBlocks, numberChunks, _allocFallback);
            externalBuffer = false;
        }
        
        void restart(size_t _blockSize, uint8_t* _buffer, size_t _bufferSize, bool _allocFallback = false)
        {
            numberBlocks = block_count_from_size(_blockSize, _bufferSize);
            size_t numberChunks = (numberBlocks + FIXED_POOL_CHUNK_SIZE - 1) / FIXED_POOL_CHUNK_SIZE;
            initialize_internal(_buffer, _blockSize, numberBlocks, numberChunks, _allocFallback);
            externalBuffer = true;
        }
        
        void* allocate()
        {
            lock.lock();

            if (nonFullChunkCount == 0) {
                lock.unlock();
                if (!allocFallback)
                    throw std::bad_alloc();
                return new uint8_t[blockSize];
            }

            size_t chunkIndex = nonFullChunks[nonFullChunkCount - 1];
            size_t bitmap = chunkBitmaps[chunkIndex];

            size_t freeBlock = unsafe_bit_scan_forward(bitmap);
            bitmap ^= (size_t)1 << freeBlock;

            chunkBitmaps[chunkIndex] = bitmap;
            nonFullChunkCount -= (bitmap == 0);
            allocatedBlocks += 1;

            lock.unlock();

            uint8_t* ptr = blocks + blockSize * (chunkIndex * FIXED_POOL_CHUNK_SIZE + freeBlock);
            return ptr;
        }

        void free(void* ptr)
        {
            if (ptr == nullptr)
                return;
            size_t index = size_t((uint8_t*)ptr - blocks) / blockSize;
            // Pointer came from new allocation
            if (index >= numberBlocks) {
                delete[] (uint8_t*)ptr;
                return;
            }

            lock.lock();

            size_t chunkIndex = index / FIXED_POOL_CHUNK_SIZE;
            size_t blockIndex = index % FIXED_POOL_CHUNK_SIZE;

            size_t bitmap = chunkBitmaps[chunkIndex];
            //This chunk now has one free block; append to stack
            if (bitmap == 0) {
                nonFullChunks[nonFullChunkCount] = chunkIndex;
                nonFullChunkCount += (bitmap == 0);
            }
            bitmap |= (size_t)1 << blockIndex;
            chunkBitmaps[chunkIndex] = bitmap;
            allocatedBlocks -= 1;

            lock.unlock();
        }
        
        size_t get_allocated_blocks() 
        {
            lock.lock();
            size_t result = allocatedBlocks;
            lock.unlock();
            return result;
        }
        
        bool empty() 
        {
            return get_allocated_blocks() == 0;
        }
        
        bool full()
        {
            return get_allocated_blocks() == numberBlocks;
        }

        size_t get_block_size() 
        {
            return blockSize;
        }
        
        size_t get_number_blocks() 
        {
            return numberBlocks;
        }
        
        // Returns the ammount of memory used by this pool, that is, the sum of the data blocks and required metadata
        size_t get_internal_allocated_memory() 
        {
            size_t numberChunks = (numberBlocks + FIXED_POOL_CHUNK_SIZE - 1) / FIXED_POOL_CHUNK_SIZE;
            return numberBlocks * blockSize + numberChunks * sizeof(size_t) * 2;
        }

        // Returns the buffer that the pool uses internally for blocks and metadata
        void get_internal_buffer()
        {
            return buffer;
        }
    };


    /*************************************************************
     *                 Variable Size Memory Pool                 *
     ************************************************************/

    // A memory pool which can allocate and deallocate any number of bytes with 
    // relatively low overhead. allocate() and free() are O(log n), with n the 
    // number of allocated segments in the pool.
    //
    // The pool is composed of blocks of a fixed 1024 bytes. It has 2 different 
    // allocation mechanisms depending on the size:
    // - >=3072 bytes: uses a binary tree to quickly find a free segment with 
    //   enough capacity for the given allocation. Each block has associated
    //   information, such as whether is free or occupied, the length of the 
    //   segment it belongs to and an index pointing to the previous segment.
    // - <3072 bytes: takes 32 contiguous blocks using the previous algorithm,
    //   and initializes a FixedMemoryPool in them with an appropiate block size.
    //   It uses a linked list to keep track of pools with available blocks for 
    //   each size. 

    // Used by the binary tree in VariableMemoryPool
    template<typename T>
    class MyPoolAlloc {
    public:

        FixedMemoryPool<>* leafPool;
        FixedMemoryPool<>* internalPool;

        typedef size_t     size_type;
        typedef ptrdiff_t  difference_type;
        typedef T* pointer;
        typedef const T* const_pointer;
        typedef T& reference;
        typedef const T& const_reference;
        typedef T          value_type;
        typedef std::true_type propagate_on_container_move_assignment;
        typedef std::true_type is_always_equal;

        template<typename X>
        using rebind = MyPoolAlloc<X>;

        MyPoolAlloc(FixedMemoryPool<>* _leafPool, FixedMemoryPool<>* _internalPool) noexcept {
            leafPool = _leafPool;
            internalPool = _internalPool;
        }

        MyPoolAlloc() noexcept {
            leafPool = nullptr;
            internalPool = nullptr;
        }

        MyPoolAlloc(const MyPoolAlloc& alloc) noexcept {
            leafPool = alloc.leafPool;
            internalPool = alloc.internalPool;
        }

        template<typename X>
        MyPoolAlloc(const MyPoolAlloc<X>& alloc) noexcept {
            leafPool = alloc.leafPool;
            internalPool = alloc.internalPool;
        }

        ~MyPoolAlloc() noexcept {};

        pointer address(reference __x) const { return &__x; }

        const_pointer address(const_reference __x) const { return &__x; }

        pointer allocate(size_type __n, const void* hint = 0) {
            if (__n * sizeof(T) <= leafPool->get_block_size())
                return reinterpret_cast<T*>(leafPool->allocate());
            return reinterpret_cast<T*>(internalPool->allocate());
        }

        void deallocate(pointer __p, size_type __n) {
            if (__n * sizeof(T) <= leafPool->get_block_size())
                leafPool->free(reinterpret_cast<uint8_t*>(__p));
            else
                internalPool->free(reinterpret_cast<uint8_t*>(__p));
        }

        size_type max_size() const {
            return SIZE_MAX;
        }

        void construct(pointer __p, const T& __val) {
            ::new(__p) T(__val);
        }

        void destroy(pointer __p) {
            __p->~T();
        }
    };

    struct VarBlockNode 
    {
        size_t isFree : 1;
        size_t firstOfSegment : 1;  // Is this block the first of a given allocation?
        size_t subSizeID : 8;  // Suballocation size used by this block, or VAR_SIZE_ID if it doesn't use suballocation
#if IS_64BIT
        size_t length : 54;
#else
        size_t length : 22;
#endif
        size_t previous;  // Index of previous segment
    };

    struct FixedBlockNode
    {
        size_t prev;
        void* blockPtr;
        size_t next;
    };

    struct FreeSegment {
        size_t length;
        size_t index;

        FreeSegment() {}
        FreeSegment(size_t _len, size_t _idx) {
            length = _len;
            index = _idx;
        }
    };

    // For memory efficiency the bTree will only store the block index, and
    // we will fetch the length from the blockNodes array.
    struct bTreeFreeSegment {
        size_t index;

        bTreeFreeSegment() {}
        bTreeFreeSegment(size_t index) {
            this->index = index;
        }
        bTreeFreeSegment(const FreeSegment& chunk) {
            this->index = chunk.index;
        }
    };

    const int VAR_SIZE_ID = 255;  // Use variable block allocation
    const int FREE_NODE_HEAD = 255;
    const size_t END_OF_LINKED_LIST = -1;
    const size_t VAR_POOL_BLOCK_SIZE = 1024;
    const size_t VAR_POOL_SUBALLOC_SEGMENT_SIZE = 32;  // Use this segment size per suballocation
    const size_t VAR_ALLOC_THRESHOLD = 5120;  // Always use variable block allocation for sizes larger than this

    // A memory pool with fixed size blocks, but it can allocate/free any number of contiguous blocks.
    // It requires about 24 bytes of additional space for each block.
    // allocate() and free() are O(log n), with n the number of blocks in the pool.
    template<typename LockType = NoLock>
    class VariableMemoryPool
    {
        size_t numberBlocks = 0;
        size_t fixedNodeHeads[256];
        uint8_t* data = nullptr;
        VarBlockNode* blockNodes = nullptr;
        FixedBlockNode* fixedNodes = nullptr;

        struct FreeSegmentComparator {
            const VariableMemoryPool& pool;
            explicit FreeSegmentComparator(const VariableMemoryPool& p) : pool(p) {}
            using is_transparent = void;

            bool operator()(const bTreeFreeSegment& lhs, const bTreeFreeSegment& rhs) const {
                if (pool.blockNodes[lhs.index].length == pool.blockNodes[rhs.index].length)
                    return lhs.index < rhs.index;
                return pool.blockNodes[lhs.index].length < pool.blockNodes[rhs.index].length;
            }

            // To erase or find a free segment if we already know its length and index 
            bool operator()(const bTreeFreeSegment& lhs, const FreeSegment& rhs) const {
                if (pool.blockNodes[lhs.index].length == rhs.length)
                    return lhs.index < rhs.index;
                return pool.blockNodes[lhs.index].length < rhs.length;
            }
            bool operator()(const FreeSegment& lhs, const bTreeFreeSegment& rhs) const {
                if (lhs.length == pool.blockNodes[rhs.index].length)
                    return lhs.index < rhs.index;
                return lhs.length < pool.blockNodes[rhs.index].length;
            }

            // To find the smallest free chunk that is at least of the given length
            bool operator()(const bTreeFreeSegment& lhs, size_t len) const {
                return pool.blockNodes[lhs.index].length < len;
            }
            bool operator()(size_t len, const bTreeFreeSegment& rhs) const {
                return len < pool.blockNodes[rhs.index].length;
            }
        };

        FixedMemoryPool<> bTreeLeafPool;
        FixedMemoryPool<> bTreeInternalPool;
        MyPoolAlloc<bTreeFreeSegment> bTreeAlloc;
        phmap::btree_set<bTreeFreeSegment, FreeSegmentComparator, MyPoolAlloc<bTreeFreeSegment>> freeSegments;
        bool allocFallback = false;
        LockType lock;

        size_t id_to_size(size_t id)
        {
            id += 1;
            if (id <= 16)
                return id;
            int bits = id / 8 - 1;
            size_t size = (8 | (id & 7)) << bits;
            return size;
        }

        size_t size_to_id(size_t size)
        {
            size -= 1;
            if (size < 16)
                return size;
            int id = unsafe_int_log2(size);
            id = (id - 2) * 8 + (size >> (id - 3) & 7);
            // Fits perfectly in one or more blocks. Using suballocation would only add overhead.
            if (id_to_size(id) % VAR_POOL_BLOCK_SIZE == 0)
                return VAR_SIZE_ID;
            return id;
        }

        void* allocate_varblock(size_t blocksRequested)
        {
            auto freeSegment = freeSegments.lower_bound(blocksRequested);
            if (freeSegment == freeSegments.end())
                return nullptr;

            size_t blockIndex = freeSegment->index;
            size_t chunkLength = blockNodes[blockIndex].length;
            freeSegments.erase(freeSegment);

            blockNodes[blockIndex].isFree = false;
            blockNodes[blockIndex].length = blocksRequested;

            if (chunkLength > blocksRequested) {
                blockNodes[blockIndex + blocksRequested].isFree = true;
                blockNodes[blockIndex + blocksRequested].length = chunkLength - blocksRequested;
                blockNodes[blockIndex + blocksRequested].previous = blockIndex;
                freeSegments.insert(FreeSegment(chunkLength - blocksRequested, blockIndex + blocksRequested));
            }
            //Update the previous link of the next chunk
            if (blockIndex + chunkLength != numberBlocks) {
                blockNodes[blockIndex + chunkLength].previous =
                    blockIndex + (chunkLength > blocksRequested ? blocksRequested : 0);
            }
            return data + blockIndex * VAR_POOL_BLOCK_SIZE;
        }

        void deallocate_varblock(size_t blockIndex)
        {
            size_t segmentLength = blockNodes[blockIndex].length;

            // Merge with next chunk
            if (blockIndex + segmentLength != numberBlocks) {
                if (blockNodes[blockIndex + segmentLength].isFree) {
                    freeSegments.erase(FreeSegment(blockNodes[blockIndex + segmentLength].length, blockIndex + segmentLength));
                    segmentLength += blockNodes[blockIndex + segmentLength].length;
                }
            }
            // Merge with previous chunk
            if (blockIndex != 0) {
                size_t previousSegment = blockNodes[blockIndex].previous;
                if (blockNodes[previousSegment].isFree) {
                    freeSegments.erase(FreeSegment(blockNodes[previousSegment].length, previousSegment));
                    segmentLength += blockNodes[previousSegment].length;
                    blockIndex = previousSegment;
                }
            }

            // Update the previous link of the next chunk
            if (blockIndex + segmentLength != numberBlocks)
                blockNodes[blockIndex + segmentLength].previous = blockIndex;

            blockNodes[blockIndex].isFree = true;
            blockNodes[blockIndex].length = segmentLength;
            freeSegments.insert({ blockIndex });
        }

        void deallocate_fixblock(void* ptr, size_t blockIndex)
        {
            while (!blockNodes[blockIndex].firstOfSegment)
                blockIndex -= 1;

            FixedMemoryPool<>* pool = (FixedMemoryPool<>*)(data + blockIndex * VAR_POOL_BLOCK_SIZE);
            size_t* metadata = (size_t*)(data + blockIndex * VAR_POOL_BLOCK_SIZE + sizeof(FixedMemoryPool<>));
            size_t node = *metadata;
            size_t sizeID = blockNodes[blockIndex].subSizeID;

            bool isFull = pool->full();
            pool->free(ptr);

            if (pool->empty()) {
                deallocate_varblock(blockIndex);

                if (fixedNodeHeads[sizeID] == node)
                    fixedNodeHeads[sizeID] = fixedNodes[node].next;

                if (fixedNodes[node].next != END_OF_LINKED_LIST)
                    fixedNodes[fixedNodes[node].next].prev = fixedNodes[node].prev;
                if (fixedNodes[node].prev != END_OF_LINKED_LIST)
                    fixedNodes[fixedNodes[node].prev].next = fixedNodes[node].next;

                // Add the fixed node to the available stack
                fixedNodes[node].next = fixedNodeHeads[FREE_NODE_HEAD];
                fixedNodeHeads[FREE_NODE_HEAD] = node;
            }
            else if (isFull) {
                // If the pool was full, now it isn't. Put it in the list of available fixed pools.
                fixedNodes[node].prev = END_OF_LINKED_LIST;
                fixedNodes[node].next = fixedNodeHeads[sizeID];
                fixedNodes[fixedNodeHeads[sizeID]].prev = node;
                fixedNodeHeads[sizeID] = node;
            }
        }

        void* allocate_fixblock(size_t sizeID)
        {
            if (fixedNodeHeads[sizeID] != END_OF_LINKED_LIST)
            {
                void* ptr = fixedNodes[fixedNodeHeads[sizeID]].blockPtr;
                FixedMemoryPool<>* pool = (FixedMemoryPool<>*)ptr;
                ptr = pool->allocate();

                // Remove the pool from the free pool list
                if (pool->full()) {
                    size_t node = fixedNodeHeads[sizeID];
                    fixedNodeHeads[sizeID] = fixedNodes[node].next;
                }
                return ptr;
            }

            void* ptr = allocate_varblock(VAR_POOL_SUBALLOC_SEGMENT_SIZE);
            if (!ptr)
                return nullptr;
            size_t blockIndex = size_t((uint8_t*)ptr - data) / VAR_POOL_BLOCK_SIZE;

            size_t chunkOverhead = sizeof(FixedMemoryPool<>) + sizeof(size_t);
            FixedMemoryPool<>* pool = new (ptr) FixedMemoryPool<>(
                id_to_size(sizeID), (uint8_t*)ptr + chunkOverhead, VAR_POOL_BLOCK_SIZE * VAR_POOL_SUBALLOC_SEGMENT_SIZE - chunkOverhead
            );

            size_t node = fixedNodeHeads[FREE_NODE_HEAD];
            fixedNodeHeads[sizeID] = node;
            fixedNodeHeads[FREE_NODE_HEAD] = fixedNodes[node].next;
            size_t* metadata = (size_t*)((uint8_t*)ptr + sizeof(FixedMemoryPool<>));
            *metadata = node;

            fixedNodes[node].next = END_OF_LINKED_LIST;
            fixedNodes[node].blockPtr = ptr;
            fixedNodes[node].prev = END_OF_LINKED_LIST;
            for (size_t i = 0; i < VAR_POOL_SUBALLOC_SEGMENT_SIZE; i++) {
                blockNodes[blockIndex + i].firstOfSegment = i == 0;
                blockNodes[blockIndex + i].subSizeID = sizeID;
            }

            ptr = pool->allocate();
            return ptr;
        }

    public:

        VariableMemoryPool()
            : bTreeAlloc(&bTreeLeafPool, &bTreeInternalPool), freeSegments(FreeSegmentComparator(*this), bTreeAlloc) {}
        VariableMemoryPool(size_t _numberBlocks, bool _allocFallback = false)
            : bTreeAlloc(&bTreeLeafPool, &bTreeInternalPool), freeSegments(FreeSegmentComparator(*this), bTreeAlloc) {
            restart(_numberBlocks, _allocFallback);
        }
        ~VariableMemoryPool() {
            delete[] data;
            delete[] blockNodes;
            delete[] fixedNodes;
        }

        void restart(size_t _poolSize, bool _allocFallback = false)
        {
            delete[] data;
            data = nullptr;
            delete[] blockNodes;
            blockNodes = nullptr;
            delete[] fixedNodes;
            fixedNodes = nullptr;

            numberBlocks = (_poolSize + VAR_POOL_BLOCK_SIZE - 1) / VAR_POOL_BLOCK_SIZE;
            data = new uint8_t[VAR_POOL_BLOCK_SIZE * numberBlocks];
            blockNodes = new VarBlockNode[numberBlocks];

            // Maximum ammount of free segments we can have is ~(numberBlocks / 2), with alternating 
            // used and free blocks, otherwise they would get merged.
            size_t maxFreeSegments = (numberBlocks + 1) / 2;

            // A leaf node of the binary tree can hold up to 1024 bytes of data. This includes space 
            // for 2 qwords or dwords for 64 or 32 bit architectures. So the number of elements per 
            // leaf node is given by kNodeValues = (1024 - sizeof(void*) * 2) / sizeof(ValueType), 
            // with a minimum value of 3. The size it occupies is kNodeValues * sizeof(ValueType) + sizeof(void*) * 2
            int kNodeValues = (1024 - sizeof(void*) * 2) / sizeof(bTreeFreeSegment);
            if (kNodeValues < 3)
                kNodeValues = 3;
            int kLeafNodeSize = kNodeValues * sizeof(bTreeFreeSegment) + sizeof(void*) * 2;
            // A non leaf node can also hold kNodeValues, but instead of 2 qwords/dwords it holds
            // kNodeValues + 3 of them. Each of them can have up to kNodeValues + 1 children.
            int kInternalNodeSize = kNodeValues * sizeof(bTreeFreeSegment) + sizeof(void*) * (kNodeValues + 3);

            // These are approximations, real count is usually much lower 
            // but we want to avoid running out of nodes in the memory pools.
            size_t kMaxLeafCount = maxFreeSegments / ((kNodeValues + 1) / 2) + 1;
            size_t kMaxInternalCount = kMaxLeafCount / ((kNodeValues + 1) / 2) + 1;

            freeSegments.clear();
            bTreeLeafPool.restart(kLeafNodeSize, kMaxLeafCount, false);
            bTreeInternalPool.restart(kInternalNodeSize, kMaxInternalCount, false);

            allocFallback = _allocFallback;
            blockNodes[0].isFree = true;
            blockNodes[0].length = numberBlocks;
            blockNodes[0].previous = 0;
            freeSegments.insert(bTreeFreeSegment(0));

            // Initialize fixed memory pools
            size_t numberFixedNodes = numberBlocks / VAR_POOL_SUBALLOC_SEGMENT_SIZE;
            fixedNodes = new FixedBlockNode[numberFixedNodes];
            for (size_t i = 0; i < numberFixedNodes; i++)
                fixedNodes[i].next = i == 0 ? END_OF_LINKED_LIST : i - 1;
            for (size_t i = 0; i < 256; i++)
                fixedNodeHeads[i] = END_OF_LINKED_LIST;
            fixedNodeHeads[FREE_NODE_HEAD] = numberFixedNodes - 1;
        }

        void* allocate(size_t size)
        {
            if (size == 0)
                return nullptr;
            size_t blocksRequested = (size + VAR_POOL_BLOCK_SIZE - 1) / VAR_POOL_BLOCK_SIZE;

            lock.lock();

            void* ptr = nullptr;
            if (size < VAR_ALLOC_THRESHOLD) {
                int id = size_to_id(size);
                if (id != VAR_SIZE_ID) {
                    ptr = allocate_fixblock(id);
                }
            }

            if (!ptr) {
                ptr = allocate_varblock(blocksRequested);
                if (ptr) {
                    size_t blockIndex = size_t((uint8_t*)ptr - data) / VAR_POOL_BLOCK_SIZE;
                    blockNodes[blockIndex].subSizeID = VAR_SIZE_ID;
                }
            }

            lock.unlock();

            if (ptr)
                return ptr;
            if (!allocFallback)
                throw std::bad_alloc();
            return new uint8_t[size];
        }

        void free(void* ptr)
        {
            if (ptr == nullptr)
                return;

            size_t blockIndex = size_t((uint8_t*)ptr - data) / VAR_POOL_BLOCK_SIZE;
            // Pointer came from new allocation
            if (blockIndex >= numberBlocks) {
                delete[] (uint8_t*)ptr;
                return;
            }

            lock.lock();

            // Check if this pointer is part of a suballocation inside a 
            // fixed memory pool or if it comes from a variable block allocation.
            if (blockNodes[blockIndex].subSizeID != VAR_SIZE_ID)
                deallocate_fixblock(ptr, blockIndex);
            else 
                deallocate_varblock(blockIndex);

            lock.unlock();
        }

        // Returns the ammount of memory used by this pool, that is, the sum of the data blocks and required metadata
        size_t get_internal_allocated_memory()
        {
            size_t usedMemory = VAR_POOL_BLOCK_SIZE * numberBlocks;  // data
            usedMemory += sizeof(VarBlockNode) * numberBlocks;  // block nodes
            usedMemory += bTreeLeafPool.get_internal_allocated_memory();
            usedMemory += bTreeInternalPool.get_internal_allocated_memory();
            usedMemory += sizeof(FixedBlockNode) * (numberBlocks / VAR_POOL_SUBALLOC_SEGMENT_SIZE);  // fixed nodes
            return usedMemory;
        }
    };
}

