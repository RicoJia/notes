/*
- Prototype
    - function has prototype (which can be thought of as class attributes in python)
        - prototype is meant to be shared
    - class (ES6+) has is still based on prototypes.
    - There's a prototype chain, which is effectively mro (method resolution order)
        - Will look up the prototype chain. If null, you are done
    - Differences w/ mro:
        - mro is always not touched after class is created. prototype is
            dynamic
        - Prototype does NOT support multiple inheritance
- New:
    - new will create a new instance of the function object, that links to the
        same prototype
    - binds this
*/

// pre ES6
function Dog(name){
    this.name = name;
}

Dog.prototype.bark = function(){
    console.log(`${this.name} is barking!`);
}

const d = new Dog( "Doggy in a function");
d.bark()
const d3 = new Dog( "another dog");
d3.bark()

// After ES6
class Doggy{
    constructor(name){
        this.name = name;
    }
    bark(){
        console.log(`${this.name} is barking!`);
    }
}
const d2 = new Doggy( "Doggy in a class");
d2.bark();

/*
- extends is creating a child class
*/
class Doggy2 extends Doggy{
    constructor(name){
        super(name);
    }
}
const d4 = new Doggy2( "Doggy4 in a class");
d4.bark();

/*
- MFR: map, filter, reduce
    - Common Higher Order functions in Functional Programming
    - Map: apply a transform on each element of the array
        - cpp equivalent: std::transform
    - Filter: Return a new array of elements, after filtering out elements by a predicate
        - cpp equivalent: std::copy_if
    - Reduce: reducing the array into a single value.
        - cpp equivalent: std::accumulate
*/
const numbers = [1,2,3,4,5];
const squared_numbers = numbers.map(num => num * num);
console.log("mapping: ", squared_numbers);
const smaller_than_3 = numbers.filter(n => n < 3);
console.log("filter: smaller than 3: ", smaller_than_3);
const sum = numbers.reduce((acc, cur) => acc + cur, 0);
console.log("reduce: sum: ", sum);