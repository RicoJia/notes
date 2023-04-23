# distutils: language = c++
# distutils: extra_compile_args = -std=c++11

cdef extern from "person.hpp":
    cdef cppclass Person:
        pass
#         Person(string name, int age) except +
#         string getName()
#         int getAge()
#         void setName(string name)
#         void setAge(int age)
#
# def create_person(name, age):
#     return Person(name, age)
#
# def get_person_name(Person person):
#     return person.getName()
#
# def get_person_age(Person person):
#     return person.getAge()
#
# def set_person_name(Person person, name):
#     person.setName(name)
#
# def set_person_age(Person person, age):
#     person.setAge(age)
#
