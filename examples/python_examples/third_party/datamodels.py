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
if __name__ == "__main__":
    test_basic_pydantic()
