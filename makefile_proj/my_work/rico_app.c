#include "dll.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*Application specific data structures*/
typedef struct person_{
    char name[32];
    int age;
    int weight;
} person_t;

static void
print_person_details(person_t *person){
    printf("Name = %s\n",   person->name);
    printf("Age = %d\n",    person->age);
    printf("weight = %d\n", person->weight);
}

/*Return 0 if a match is found, else return a non zero number*/
int compare_name(void* data, void* key){
	person_t* p = (person_t*)data;
	return  strcmp(p -> name, (char*)key);
} 

void find_person(dll_t* dll, char* name){
	register_comparison_callback(dll, compare_name); 
	person_t* p = (person_t*)(dll_search_by_key(dll, name));
	if (p != NULL){
		printf("Person found! \n"); 
		print_person_details(p); 
	}
	else{
		printf("Person not found under name %s \n", name); 
	}
}

static void
print_person_db(dll_t *person_db) {

    if(!person_db || !person_db->head) return;

    dll_node_t *head = person_db->head;
    person_t *data = NULL;

    while(head){
        data = head->data;
        print_person_details(data);
        head = head->right;    
    }
}

int 
main(int argc, char **argv){

    person_t *person1 = calloc(1, sizeof(person_t));
    strncpy(person1->name, "Abhishek", strlen("Abhishek"));
    person1->age = 31;
    person1->weight = 75;
    person_t *person2 = calloc(1, sizeof(person_t));
    strncpy(person2->name, "Joseph", strlen("Joseph"));
    person2->age = 41;
    person2->weight = 70;
    person_t *person3 = calloc(1, sizeof(person_t));
    strncpy(person3->name, "Jack", strlen("Jack"));
    person3->age = 29;
    person3->weight = 55;

    /*Create a new Linked List*/

    dll_t *person_db = get_new_dll();
    add_data_to_dll(person_db, person1);
    add_data_to_dll(person_db, person2);
    add_data_to_dll(person_db, person3);

    print_person_db(person_db);
   
	// test search
	find_person(person_db, "Joseph"); 

	//test utils
	drain_dll(person_db); 
	printf("Database successfully deleted? %d \n", 1+is_dll_empty(person_db)); 
    return 0;
}
