Readme for Buffer Management Module

1. Arch

  The BMM is composed of a kernel module and a user library.

2. Kernel Module

  The bmm.ko should be inserted just after Linux boots up to preserve large
  physical continous memory for HW video accelerator. A device node should be
  created by the following commands, in case udev rules fails to do so:

  mknod /dev/bmm c 10 94

3. User Library

  The user application should include bmm_lib.h in the source code and link to
  libbmm.a or libbmm.so.

4. APIs exported to user

  - int bmm_init()
    Open BMM device (/dev/bmm);
    Return the file descriptor (>0) if successfully; or return NULL in error;
    It's not necessary to call bmm_init() directly since it will be called
    automatically by the other APIs;
    if /dev/bmm does not exist, this function will try to create it.

  - void bmm_exit()
    Close BMM device (/dev/bmm);
    If the application doesn't call bmm_exit() directly, the BMM file handler
    will be closed automatically when app exits.

  - void * bmm_malloc(unsigned long size, int attr)
    Allocate a block of physical continuous memory;
    Return the virtual address of the allocated physical continuous memory
    (>0) if successfully; or return NULL in error;
    size: memory size with a unit of byte, auto-expanded to 4KB align;
    attr: memory attributes when mmap'ing;
      #define BMM_ATTR_DEFAULT       (0)      /* cacheable bufferable */
      #define BMM_ATTR_NONBUFFERABLE (1 << 0) /* non-bufferable */
      #define BMM_ATTR_NONCACHEABLE  (1 << 1) /* non-cacheable */

      /* Note: extra attributes below are not supported yet! */
      #define BMM_ATTR_HUGE_PAGE     (1 << 2) /* 64KB page size */
      #define BMM_ATTR_WRITETHROUGH  (1 << 3) /* implies L1 Cacheable */
      #define BMM_ATTR_L2_CACHEABLE  (1 << 4) /* implies L1 Cacheable */

    /* Note: the virtual address is NOT shared by any forked sub-process
             to share this memory, please call bmm_attach and bmm_detach */

  - void bmm_free(void *vaddr)
    Free the memory allocated by bmm_malloc;
    "vaddr" should be an valid value returned by bmm_malloc(size, attr).

  - void * bmm_attach(void *paddr, unsigned long len)
    Attach the block of physical continuous memory into current process;
    It shares the same attributes set in bmm_malloc in allocating process;
    Return the virtual address of the specified physical continuous memory
    (>0) if successfully; or return NULL in error;
    paddr: the physical address of the memory, which should be 4KB align and
      transferred from the allocating process to the current process;
    len: the size of the memory block the current process want to share
      from the allocating process, auto-expanded to 4KB align.

  - void bmm_detach(void *vaddr, unsigned long len)
    Detach the block of physical continuous memory from current process;
    vaddr: the valid virtual address return by bmm_attach(paddr, length);
    len: the same valid value as bmm_attach(paddr, length).

  - void * bmm_get_vaddr(void *paddr)
    Get vitual address from existing physical address;
    Return the virtual address mmap'ed from "paddr" (>0) if successfully;
    Or return NULL in error;
    "paddr" should be an valid value returned by bmm_get_paddr(vaddr).

  - void * bmm_get_paddr(void *vaddr)
    Get physical address from existing virtual address, the memory must come
    form bmm_malloc, otherwise NULL is returned;
    Return the physical address mmap'ed to "vaddr" (>0) if successfully;
    Or return NULL in error;
    "vaddr" should be an valid value returned by bmm_malloc(size, attr).

  - void * bmm_get_kern_paddr(void *vaddr, unsigned long size)
    Get physical address from existing virtual address, no matter the memory
    is got via bmm_malloc or form kernel space.
    Return the physical address mmap'ed to "vaddr" (>0) if successfully;
    Or return NULL in error;
    "vaddr" should be an valid value returned by bmm_malloc(size, attr).

  - int bmm_get_mem_attr(void *vaddr)
    Return the attributes of the memory block;
    See bmm_malloc above for attribute details.

  - int bmm_set_mem_attr(void *vaddr, int attr)
    Modify memory attributes;
    Return the new attributes of the memory block;
    See bmm_malloc above for attribute details.
    /* Note: this function is not supported yet! */

  - unsigned long bmm_get_mem_size(void *vaddr)
    Get memory block size from existing virtual address;
    Return the memory block size in bytes (>0) if successfully;
    Or return NULL in error;
    "vaddr" should be an valid value returned by bmm_malloc(size, attr).

  - unsigned long bmm_get_total_space()
    Get total memory size reserved by BMM in a unit of byte;
    Return the total memory block size in bytes (>=0);

  - unsigned long bmm_get_free_space()
    Get current available memory size (not allocated yet) in a unit of byte;
    Return the available memory block size in bytes (>=0);

  - unsigned long bmm_get_allocated_space()
    Get the memory size allocated by the current process in a unit of byte;
    Return the allocated memory block size in bytes (>=0);

  - void bmm_flush_cache(void *vaddr, int dir)
    Flush cache of vaddr, which is useful if vaddr is cacheable;
    dir: same as consistent_sync.

  - void *bmm_dma_memcpy(void *dest, const void *src, size_t n)
    DMA version of memcpy;
    dest: dest virtual address, should be 32 bytes align & physical continuous;
    src : src  virtual address, should be 32 bytes align & physical continuous;
    n   : memory size in bytes;
    /* Note: this function is obsolete already! */

  - void bmm_dma_sync()
    Wait until the above bmm_dma_memcpy has finished;
    /* Note: this function is obsolete already! */

5. Driver IOCTLs

  - User lib APIs have corresponding IOCTLs to BMM driver
  - BMM driver is opened automatically whichever API is called

6. Limitation

  - Any memory allocation will be expanded to 4KB aligned;
  - First fit allocation algorithm is used currently;
  - Alloc/free should be used in pairs.

7. Example code

  - See bmm_test.c for a detailed example.

