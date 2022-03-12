#!/usr/bin/env python3
import os

ret = os.system("git add .")

if(ret!=0):
    print("Error running the previous command")

message = input("Please enter the commit message: ")

ret = os.system("git commit -m ; message")

ret = os.system("git push origin ; git push personal")