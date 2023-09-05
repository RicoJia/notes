"use strict";
/**
 * In Javascript we can't replace the string index directly, like
    Also, we can't do name_i[1:-1]
    name_i[0]=name_i[0].toUpperCase();
 */
const test_string_replace = () => {
    // name is a host object's name, so otherwise it will overwrite
    var name_i = "hello"
    name_i = `${name_i[0].toUpperCase()}${name_i.slice(1, name_i.length)}`;
    name_i = name_i.charAt(0).toUpperCase() + name_i.slice(1);
    console.log(name_i.slice(0,3));

}

const test_remove_spacing = () => {
    const originalString = "   Hello, world!   ";
    console.log("trimmed: ", originalString.trim());
}

test_remove_spacing();
