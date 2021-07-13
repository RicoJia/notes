#ifndef __DLL__
#define __DLL__
/*Header file for Doubly Linked List*/
// data structure: 
// head-> node1 (right) -> node2 (left) (right) -> ...

typedef struct dll_node_{
    void *data;
    struct dll_node_ *left;
    struct dll_node_ *right;
} dll_node_t;

typedef struct dll_{
    dll_node_t *head;
	int (*search_by_key_ptr)(void*, void*);
	int (*compare)(void*, void*);
} dll_t;


/* Public function declaration for registering search by key function*/
void register_key_match_callback(dll_t* dll, int (*search_by_key_ptr)(void*, void*));
/* Public function declaration for registering comparison function*/
void register_comparison_callback(dll_t* dll, int (*compare)(void*, void*));

/* Public function declaration for searching by a key */
void* dll_search_by_key(dll_t* dll, void* key); 

/* Public Function declaration to create and return
 * a new empty doubly linked list*/
dll_t *
get_new_dll();

/*Public Function declaration to add a new application
 * data to DLL*/
/* return 0 on success, -1 on failure*/
int
add_data_to_dll (dll_t *dll, void *app_data);
#endif
