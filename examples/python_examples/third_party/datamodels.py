#!/usr/local/bin/python3

def test_basic_pydantic():
    from pydantic import BaseModel
    class User(BaseModel):
        id: int
            # there's a default value to this
        name = "test"

    u = User(id = 123, name="101")
    print(u.id, type(u.id))

    # can see which fields have been set.
    print(u.__fields_set__)

    # see dict , and json str
    print(u.dict(), type(u.json()))

    # Return JSON schema, a dict
    print(u.schema, type(u.schema()))

def test_dataclass():
    """
    1. 
    """
    import datetime as dt
    from dataclasses import dataclass
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

if __name__ == "__main__":
    test_basic_pydantic()
