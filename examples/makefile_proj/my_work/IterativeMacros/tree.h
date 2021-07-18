#ifndef __TREE__
#define __TREE__
#include<stdio.h>
#include <memory.h>
/*Define the in-order non recursive BST traversal function */
typedef struct tree_node {              
    struct tree_node *left;
    struct tree_node *right;
    struct tree_node *parent;
    int data;
} tree_node_t;


typedef struct tree {
    tree_node_t *root;
} tree_t;

tree_t* init_tree(void);

tree_node_t* init_tree_node(int n);

/*Pre-requisites functions to write iterative 
 * macros for a tree.*/
tree_node_t* get_left_most(tree_node_t* ptr){
//	printf("%d\n", ptr -> data); 
	if (ptr == NULL) return NULL; 
	for (; ptr -> left != NULL; ptr = ptr ->left ){}
	return ptr; 
}

tree_node_t* get_next_inorder_succ(tree_node_t* ptr){
	// workflow
	// if the parent is not null, and the child has no left child: 
	// return right child's bottom left or its parent 
	// else, return the right child's left most, or its right child.
	// If right child does not have any children, go all the way back, to the parent of the node which has the first left child
	
	printf("%u\n", ptr -> data); 
	// You're root
	if (ptr -> parent == NULL ){
		return get_left_most(ptr -> right); 
	}
	//left child
	if (ptr == ptr -> parent -> left){
		if (ptr -> right != NULL) return get_left_most(ptr -> right); 
		else return ptr -> parent; 
	}
	// right child with at least a child  
	if (ptr -> left || ptr -> right){
		if (ptr -> left) return get_left_most(ptr -> left); 
		else return ptr -> right; 
	}
	//right child with no children, but with at least a parent 
	tree_node_t* p = ptr -> parent; 
	tree_node_t* gp = p -> parent; 
	while (gp && gp -> left != p){
		p = gp; 
		gp = p -> parent; 
	}
	return gp; 
}

#define ITERATE_BST_BEGIN(tree, treenodeptr){\
	tree_node_t* _next = NULL; \
	for(treenodeptr = get_left_most(tree -> root); treenodeptr; treenodeptr = _next){ \
		_next = get_next_inorder_succ(treenodeptr); 
#define ITERATE_BST_END }}
#endif
