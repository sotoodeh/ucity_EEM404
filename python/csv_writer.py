#!/usr/bin/env python

'''
    Example 4:
    program to write student information to a .csv file
'''

import csv


def main():
    with open('my_csv.csv', 'w') as csvfile:
        headers = ["name", "email", "phone"]
        writer = csv.DictWriter(csvfile, fieldnames=headers)
        writer.writeheader()

        for i in range(0, 3):
            name_ = raw_input("Please enter your name:> ")
            email_ = raw_input("Please enter your email:> ")
            phone_ = raw_input("Please enter your phone number:> ")
            writer.writerow({'name': name_, 'email': email_, 'phone': phone_})


if __name__ == "__main__":
    main()
