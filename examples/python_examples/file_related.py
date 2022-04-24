#!/usr/bin/python3
########### Zip file ###########
# writing and reading from zip file
def zip_file_related(): 
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

    # append a file in the zip under another name. Just append, not removing
    another_hello_file = "another_hello_file.txt"
    with ZipFile(file_name, 'w') as f:
        f.writestr(another_hello_file, another_hello_file_content)
        f.close()


    def print_zip(file_name):
        with ZipFile(file_name, 'r') as f: 
            for info in f.infolist(): 
                print(info.filename)
                print(f.read(info.filename))
            f.close()

    # update just one file - we have to recompress and delete
    import os
    def update_one_file(src_name, target_name, replacement_content): 
        """
        Add or update target in a zip file with replacement_content
        src_name: zip file to be updated
        target_name: file inside src_name to be added/replaced
        replacement_content: string to be substituted in
        """
        tmp = src_name+"tmp"
        with ZipFile(src_name, "r") as infile, ZipFile(tmp, "w") as outfile: 
            for info in infile.infolist(): 
                if info.filename != target_name: 
                    outfile.writestr(info.filename, infile.read(info.filename))
                else: 
                    outfile.writestr(info.filename, replacement_content)
            infile.close()
            outfile.close()
        os.remove(src_name)
        # os.rename(old_name, new_name)
        os.rename(tmp, src_name)

    print("before updating")
    print_zip(file_name)
    print("now let's update just one file")
    update_one_file(file_name, another_hello_file, "around the world")
    print_zip(file_name)

########## argparse ###########
import argparse
# - regualr args, with default help msg. When you use it, do python file.py --src_dir SRC --dst_dir DIR, cannot omit --src_dir
# parser = argparse.ArgumentParser()
# # # custom help msg
# parser.add_argument("--src_dir", dest="src_dir", type=str, default=None, required=True)
# parser.add_argument("--dst_dir", dest="dst_dir", type=str, default=None, required=True)
# parser.add_argument("-v", "--verbose", type=int, required=False, default=0, help="verbose output.")
# # adds boolean
# parser.add_argument('-o', '--output', action='store_true', help=DOC)
# args = parser.parse_args()

# # - adds custom msg
# parser = argparse.ArgumentParser(add_help=False)
# DOC = f"""
# - This script adds snapshots to old spoofs from new spoofs, so the old spoofs can work with the store-geometry-editor
#     - Assume src_dir contains newer spoofs of the exact same store geometry as dst_dir
#     - If the two directories exist, we will append 'requests/GET/snapshot' from each spoof in src_dir, to each spoof in dst_dir
#     """
# # - add positional args, so you can do python file.py SRC DIR, no need for --src_dir
# parser.add_argument("src_dir", type=str, default=None)
# parser.add_argument("dst_dir", type=str, default=None)
# # will show a "flat" version of the DOC, with the usage already
# parser.add_argument('-h', '--help', action='help', default=argparse.SUPPRESS,
#                     help=DOC)
# args = parser.parse_args()


# - exclusive argparse
# parser = argparse.ArgumentParser()
# group = parser.add_mutually_exclusive_group(required=True)  #THIS IS GREAT!
# group.add_argument("--path", type=str, help="path to directory with labeled tracker bag files in it")
# parser.add_argument("--subdirectories", required=False, action="store_true", help="include subdirectories when searching a path")
# args = parser.parse_args()
# if args.path is not None: 
#   with open("path", 'r') as input_file: 
#     config = yaml.safe_load(input_file)



# ########## Misc ###########
# print(__file__)

def os_basics(): 
    # TODO: to organize
    import os
    os.path.join("/path_a", "dir_without_/")
    os.chdir(persistence_folder)    #change directory to that file.
    # check if dir exists, remove (remove dir has to use shutil.rmtree())
    os.path.isdir("")
    if not os.path.exists('my_folder'):
        os.remove(file_path)
        os.makedirs('my_folder')
        os.rmdir(dir_path)  # empty dir
        shutil.rmtree(folder)   #folder and content

    #write to file
    with open("file.txt", 'w') as file:   #open is a context manager
       file.write("Hello World")

    # list all files, this is a list
    os.listdir(path)
    os.rename(name)
