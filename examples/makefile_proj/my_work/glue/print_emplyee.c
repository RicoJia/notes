// #ifndef __PRINT_EMPLYEE__
// #define __PRINT_EMPLYEE__

#include <stdio.h>

typedef struct emp_ {
  char name[30];
  unsigned int salary;
  char designation[30];
  unsigned int emp_id;
  glthread_node_t glnode;
} emp_t ;

typedef struct glthread_node_t{
	struct glthread_node_t* left; 
	struct glthread_node_t* right; 
} glthread_node_t; 

typedef struct dll{
	glthread_node_t* head; 
	unsigned int offset; 
	int (*key_match)(void*, void*); 
	int (*comparison_fn)(void*, void*); 
}gldll_t; 

/*Function to print employee details*/
void print_employee_info(emp_t *emp){
    printf("Employee name = %s\n", emp->name);
    printf("salary = %u\n", emp->salary);
    printf("designation = %s\n", emp->designation);
    printf("emp_id = %u\n", emp->emp_id);
}

/*Implement the below function*/
  void print_emp_details(glthread_node_t *glnode){
		
  }

int main (int argc char**argv){
/*Creating one employee object*/
    emp_t *emp  = calloc(1, sizeof(emp_t));
    strncpy(emp1->name, "Neha", strlen("Neha"));
    emp1->salary = 50000;
    strncpy(emp1->designation, "HR", strlen("HR"));
    emp1->emp_id = 21;

/*Invoking fn to print employee details*/
    print_emp_details (&emp->glnode);

	return 0; 
}

