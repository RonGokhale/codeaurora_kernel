#ifdef __KERNEL__
# ifdef CONFIG_X86_32
#  include "unistd_32.h"
# else
#  include "unistd_64.h"
#  include "unistd_64_compat.h"
# endif
#else
# ifdef __i386__
#  include "unistd_32.h"
# elif __LP64__
#  include "unistd_64.h"
# else
#  include "unistd_x32.h"
# endif
#endif
