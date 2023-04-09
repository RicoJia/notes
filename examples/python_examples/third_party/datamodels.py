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


if __name__ == "__main__":
    # test_basic_pydantic()
    test_dataclass()
