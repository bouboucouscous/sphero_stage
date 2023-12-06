import re

string = "/robot_4123/odom"

# Define a regular expression pattern to match the number
pattern = re.compile(r"/robot_(\d+)/odom")

# Use the pattern to search for a match in the string
match = pattern.search(string)

# Check if a match is found
if match:
    # Extract the number from the matched group
    number = match.group(1)
    print("Extracted number:", number)
else:
    print("No match found.")
