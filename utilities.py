import json

def output_data(data, filename):
    # Serializing json
    json_object = json.dumps(data, indent=4)
    
    # Writing to sample.json
    with open(filename, "w") as outfile:
        outfile.write(json_object)


def import_data(data, filename):
    # Open and read the JSON file
    with open(filename, 'r') as file:
        data = json.load(file)
    return data