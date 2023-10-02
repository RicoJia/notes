const test_enum = () => {
    // Enum class can be exported
    enum DriverCarStatus {
        POWER_COMM_OK = 0,
        GROUP_OPERATION = 1,
        UP_DIRECTION = 2,
        DOWN_DIRECTION = 3,
        DOOR_FULLY_OPEN = 4,
        DOOR_FULLY_CLOSED = 5,
        FIRE_SERVICE = 6,
        CODE_BLUE = 7,
        INSPECTION_OPERATIOM = 8,
        REAR_DOOR_FULLY_OPEN = 9, // TODO: Test this
        REAR_DOOR_FULLY_CLOSED = 10, // TODO: Test this
    }

    let arr3 = Array.from({length: 16}, (_, i) => i);
    for (let i = 0; i < arr3.length; i++) {
        // Can refer to DriverCarStatus[i].
        const status: string | undefined = DriverCarStatus[i];
        console.log(status, i)
    }

    // Specify Types
    // const status: string | undefined = DriverCarStatus[i];
}
// test_enum();

/**
 * Type: different from class!
 *  - cannot be instantiated. Only lists method types and vars
 */
const test_type = () => {
    type Dog = {
        bark: () => void;
        name: string;
    };
    const d: Dog = {
        bark: () => {console.log("bark");},
        name: "tracker"
    };
    d.bark();
}
test_type();

/**
 * Partial makes all fields of a type optional
 *  - works on both class and type
 */
const test_partial = () => {
    type Foo = {
        field1: string;
        field2: number;
    };

    const f: Partial<Foo> = {field1: "foo"};
    console.log(f);

    class Bar{
        field1: string;
        field2: number;
    }
    const b: Partial<Bar> = {field1: "bar"};
    console.log(b);
}
// test_partial();

const test_optional_property = () => {
    type Person = {
        name: string;
        age?: number;
    };
    const p: Person = {name: "foo"};
    console.log(p);
}
test_optional_property();