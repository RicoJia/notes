#include "dll.h"
#include <stdlib.h>
#include <memory.h>

/*More functions*/
int  /*return 0 on success and -1 on failure*/
remove_data_from_dll_by_data_ptr (dll_t *dll, void *data){
	// workflow
	// ptr traverses from head node 
	// if ptr doesn't have data, keep moving. 
	// Else, make ptr point to child, tmp_ptr point to the current node
	// Connect the parent node to the child node; free data then free the ptr. 
	dll_node_t* ptr = dll->head; 
	while (ptr != NULL){
		if (ptr->data == data){
			dll_node_t* tmp_ptr = ptr; 
			ptr = ptr -> right; 
			tmp_ptr -> left -> right = ptr;
			ptr -> left = tmp_ptr -> left; 
			free(tmp_ptr); 
			break; 
		}	
	}
    return 0;
}

/*return 0 if empty, -1 if not empty*/
int
is_dll_empty (dll_t *dll){
	//Workflow
	//if head is null, then empty. Else not
	if (dll->head == NULL) return 0; 
	else
		return -1;
}

/* delete all nodes from a dll, but do not free appln data*/
void
drain_dll (dll_t *dll){
	// Workflow
	// while dll is not empty
	// make ptr point to the word from head
	// connect the child to head
	// free the current pointer
	while (is_dll_empty(dll) != 0)	{
	//for (int i = 0; i < 3; ++i){
		dll_node_t* ptr = dll -> head ; 
		dll -> head = ptr -> right; 
		free (ptr);		// I tripped at a null pointer check
	}
}

