import importlib
import sys

statsFiles = importlib.import_module(sys.argv[1].rsplit(".",1)[0])

#print statsFiles.files
print sys.argv[1].rsplit(".",1)[0].rsplit("statsFiles",1)[1]
