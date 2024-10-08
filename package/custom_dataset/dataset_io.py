import json
import jsbeautifier
import multiprocessing

class DatasetIO(object):
    def __init__(self):
        pass

    # Read necessary info from dataset
    def JSON_read(self, file_path):
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data

    # Write new json file to
    def JSON_write(self, file_path, width, height, pose, dcs0, dcs1, dcs2, dcs3, info): 
        image_dataset = dict()

        image_dataset["info"]   = info
        image_dataset["width"]  = width
        image_dataset["height"] = height
        image_dataset["pose"]   = pose

        image_dataset["dcs0"] = dcs0
        image_dataset["dcs1"] = dcs1
        image_dataset["dcs2"] = dcs2
        image_dataset["dcs3"] = dcs3

        with open(file_path, 'w', encoding='utf-8') as file:
            json.dump(image_dataset, file, indent="\t")
    
    def JSON_write_single(self, file_path, img):
        image = dict()
        image["image"] = img

        with open(file_path, 'w', encoding='utf-8') as file:
            json.dump(img, file, indent="\t")

    def JSON_beauty(self, file_path):
        data_json = self.JSON_read(file_path)
        
        # Beautify
        options = jsbeautifier.default_options()
        options.indent_size = 4
        options.max_preserve_newlines = 1
        options.end_with_newline = True
        options.wrap_line_length = -1
        options.break_chained_methods = True

        with open(file_path, 'w') as out_json_file:
            out_json_file.write(jsbeautifier.beautify(json.dumps(data_json, sort_keys=False), options))

# Function to beautify a single file, used by multiprocessing
def beautify_file(file_path):
    dataset = DatasetIO()
    dataset.JSON_beauty(file_path)
    print(f"Processed {file_path}")

# Function to process multiple files in parallel
def beautify_files(file_list, num_workers):
    with multiprocessing.Pool(processes=num_workers) as pool:
        pool.map(beautify_file, file_list)

if __name__ == '__main__':
    MAX_WORKERS = multiprocessing.cpu_count()
    file_path = '/home/pata/Documents/dcs_logger/dataset/tmp/img_'

    # File index input
    start_index = int(input("Enter the start index: "))
    end_index = int(input("Enter the end index: "))
    
    file_list = [file_path + str(i) + '.json' for i in range(start_index, end_index+1)]

    print(f"Total {len(file_list)} images to process.")
    num_workers = min(MAX_WORKERS, len(file_list))
    print(f"{num_workers} CPUs are used")

    beautify_files(file_list, num_workers)