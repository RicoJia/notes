/// \file
#include "dll.h"
#include <stdlib.h>

void register_key_match_callback(dll_t* dll, int (*func)(void*, void*)){
	dll->search_by_key_ptr = func; 
}
void register_comparison_callback(dll_t* dll, int (*func)(void*, void*)){
	dll -> compare = func; 
}

void* dll_search_by_key(dll_t* dll, void* key){
	// work flow
	// iterate thru the whole dll, 
	// if compare is 0, return data
	if (dll == NULL || dll -> head ==NULL) return;
	dll_node_t* ptr = dll -> head; 
	while (ptr != NULL){
		if (dll->compare(ptr->data, key) == 0)
			return (void*)ptr -> data;
		ptr = ptr -> right; 
	}
	return NULL; 
}

dll_t* get_new_dll(){
	dll_t* dll_ptr = (dll_t*) malloc(sizeof(dll_t)); 
	dll_ptr -> head = NULL; 
	return dll_ptr;
}

int add_data_to_dll(dll_t*dll, void* app_data){
	//check if pointers are valid
	if (dll == NULL || app_data == NULL) return 1; 

	// make a new node 
	dll_node_t* new_node = (dll_node_t*) malloc(sizeof(dll_node_t));
	new_node ->data = app_data;
	new_node->right= NULL;	
	// add to head of dll, if there's no head yet. 
	if (dll->head == NULL){
		new_node->left= NULL; 
		dll->head = new_node; 
		return 0;
	}
	
	// Else, make the head this new node,and connect the head to the old head. 
	new_node -> right = dll ->head; 
	dll->head->left = new_node; 
	dll->head = new_node; 
	return 0; 
}
