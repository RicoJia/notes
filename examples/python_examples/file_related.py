########### Zip file ###########
# writing and reading from zip file
from zipfile import ZipFile
file_name = "mi_archivo.zip"
hello_file = "hello.txt"
with ZipFile(file_name, 'w') as f: 
    f.writestr(hello_file, "hello".encode("utf-8"))
    f.close()

# append another file
another_file = "another_file.txt"
with ZipFile(file_name, 'a') as f: 
    f.writestr(another_file, "another file~!!!")
    f.close()

# print all info and zip content
with ZipFile(file_name, 'r') as f: 
    for info in f.infolist(): 
        if (info.filename == hello_file): 
            another_hello_file_content = f.read(hello_file)
    f.close()

# append a file in the zip under another name
another_hello_file = "another_hello_file.txt"
with ZipFile(file_name, 'a') as f:
    f.writestr(another_hello_file, another_hello_file_content)
    f.close()


with ZipFile(file_name, 'r') as f: 
    for info in f.infolist(): 
        print(info.filename)
        print(f.read(info.filename))
    f.close()

