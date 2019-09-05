#ifndef VMEMORY_H
#define VMEMORY_H

#include <Windows.h>
#include <stdint.h>

namespace tsi
{
  namespace ips
  {
    template<class T>
    class VMemory
    {
    protected:
      void * p_buf_;
      uint64_t sz_;

    public:
      VMemory()
      {
        p_buf_ = NULL;
        sz_ = 0;
      }

      VMemory(uint64_t sz)
      {
        p_buf_ = NULL;
        sz_ = 0;
        resize(sz);
      }

      ~VMemory()
      {
        clear();
      }

      void clear()
      {
        if (p_buf_) ::VirtualFree(p_buf_, 0, MEM_RELEASE);
        p_buf_ = 0;
        sz_ = 0;
      }

      void resize(uint64_t sz)
      {
        clear();
        if (sz > 0)
        {
		      p_buf_ = ::VirtualAlloc(0, (size_t) sizeof(T)*(size_t)sz, MEM_COMMIT, PAGE_READWRITE);
          if (p_buf_) sz_ = sz;
        }
      }

      T * data(){return (T *) p_buf_;}
      uint64_t size(){return sz_;}
    };
  }
}

#endif