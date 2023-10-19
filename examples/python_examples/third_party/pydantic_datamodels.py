#!/usr/local/bin/python3

def test_basic_pydantic():
    from pydantic import BaseModel,ValidationError 
    class User(BaseModel):
        id: int
            # there's a default value to this
        name = "test"

    # 1. Create a simple model. You can view dict, JSON
    u = User(id = 123, name="101")
    print(u.id, type(u.id))
    # can see which fields have been set.
    print(u.__fields_set__)
    # see dict , and json str
    print(u.dict(), type(u.json()))
    # Return JSON schema, a dict
    print(u.schema, type(u.schema()))

    # 2 Validation
    user = {
        "id": 456,
        "name": "hola",
    }
    try: 
        user_U = User(**user)
    except ValidationError as e:
        print(e)
    

def test_dataclass():
    """
    1. basic example
    2. Can use fields for default values creation
    3. Must specify type
    4. asdict(instance) can be handy
    5. create dataclass from dict
    """
    import datetime as dt
    from dataclasses import dataclass, asdict
    from marshmallow import Schema, fields

    @dataclass      #class deco: you can basically set the attributes right? What does this do?
    class Album:
        title: str
        release_date: dt.date

    class AlbumSchema(Schema):
        title = fields.Str()
        release_date = fields.Date()

    album = Album("Beggars Banquet", dt.date(1968, 12, 6))
    schema = AlbumSchema()
    data = schema.dump(album)
    data  # {'release_date': '1968-12-06', 'title': 'Beggars Banquet'}

    # must have type
    @dataclass
    class Foo:
        name = ""
        age = 12
    try:
        Foo(name="f", age=12) # .asdict()
    except TypeError as e:
        #TODO Remember to remove
        print(f'Must specify type in dataclass variables. otherwise see: {e}')

    print(f'asdict: {asdict(album)}') 

    album_dict = {"title": "hello", "release_date":dt.datetime.now().date()}
    a = Album(**album_dict)
    print(f'can create dataclass from dictionary using **: {a}')

def test_custom_pydantic():
    import pydantic, re
    from enum import Enum

    def to_camel(string: str) -> str:
        return re.sub(r'_([a-z])', lambda x: x.group(1).upper(), string)
    class Color(Enum):
        RED = "red_value"
        GREEN = "green_value"
        BLUE = "blue_value"
    class ConfigBaseModel(pydantic.BaseModel):
        # Needc to define a Config class
        class Config:
            # If you change a field's value, it will be validated again
            validate_assignment = True
            # Aliases
            allow_population_by_field_name = True
            alias_generator = to_camel
            # without it, it will show as color=<Color.RED: 'red_value'>
            # With it, it will show as color=red_value
            # However, you need "red_value" to be correct in the input json, anyways
            use_enum_values = True

    class Car(ConfigBaseModel):
        id: int
        brand: str
        color: Color
    
    try:
        input_data = {"id": "invalid id", "brand": "hola"}
        model = Car(**input_data)
    except pydantic.error_wrappers.ValidationError:
        pass

    input_data = {"id": 12345, "brand": "hola", "color": "red_value"}
    model = Car(**input_data)
    #TODO Remember to remove
    print(f'model: {model}')

if __name__ == "__main__":
    # test_basic_pydantic()
    # test_dataclass()
    test_custom_pydantic()
