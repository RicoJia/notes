import example_pb2
import scalar_types_pb2

pen = scalar_types_pb2.Person()
pen.age = -2
pen.height=180

bytestr = pen.SerializeToString()
print(bytestr)
print(type(bytestr))

pen_read = example_pb2.Pen().FromString(bytestr)
print(pen_read)

