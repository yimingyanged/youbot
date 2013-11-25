
/*--------------------------- Constants and macros -------------------------*/

/* Following defines TRUE and FALSE if not previously defined. */
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define MAX(x,y)  (((x) > (y)) ? (x) : (y))
#define MIN(x,y)  (((x) < (y)) ? (x) : (y))
#define ABS(x)    (((x) < 0) ? (-(x)) : (x))


/* Given the name of a structure, NEW allocates space for it and returns
     a pointer to the structure.
*/
#define NEW(s,pool) ((struct s *) MallocPool(sizeof(struct s),pool))


/*---------------------------- Structures -------------------------------*/

/* Information for defining a storage pool.
*/
typedef struct PoolSt {
  int remaining;  /* Number of bytes left in current block of storage. */
  char *base;     /* Pointer to base of current block of storage. */
  char *loc;      /* Current location in block to next allocate memory. */
} *Pool;


/*----------------------- External function prototypes -------------------------*/

void FatalError(char *msg);
Pool CreatePool();
void *MallocPool(int size, Pool pool);
void FreePool(Pool pool);
