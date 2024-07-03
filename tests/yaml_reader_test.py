#!/usr/bin/env python3

import yaml
from munch import Munch, munchify, unmunchify


class YAMLReader(Munch):
    """Read YAML files and store the data in an attribute style access object."""

    def __init__(self, file_path):
        """Initialize the YAML Reader"""
        super().__init__()
        self.read(file_path)  # Automatically read YAML data on initialization

    def read(self, file_path=None):
        """Read YAML data from file and munchify it"""
        if file_path is None:
            raise ValueError("YAML: No file path provided")
        try:
            with open(file_path, "r") as file:
                print(f"YAML: Loading file '{file_path}'")
                self.update(munchify(yaml.safe_load(file)))
        except Exception as e:
            print(f"YAML: Failed to load file '{file_path}': {e}")

    def write(self, file_path=None):
        """Write the YAML data to a file"""
        if file_path is None:
            raise ValueError("YAML: No file path provided")
        try:
            with open(file_path, "w") as file:
                yaml.safe_dump(unmunchify(self), file)
                print(f"YAML: File successfully written to '{file_path}'")
        except Exception as e:
            print(f"YAML: Failed to write file '{file_path}': {e}")


def test_yaml_reader():
    yaml_file = "test.yaml"
    initial_data = {
        "name": "John Doe",
        "age": 30,
        "is_student": False,
        "grades": [85, 90, 78],
        "address": {"street": "123 Main St", "city": "Anytown", "zipcode": "12345"},
        "contacts": [{"type": "email", "value": "john.doe@example.com"}, {"type": "phone", "value": "123-456-7890"}],
    }

    try:
        # Write initial data to YAML file
        with open(yaml_file, "w") as file:
            yaml.safe_dump(initial_data, file)
            print(f"YAML: Sample file created: {yaml_file}")

        # Initialize YAMLReader with the YAML file
        reader = YAMLReader(yaml_file)

        # Check loaded attributes
        assert reader.name == "John Doe"
        assert reader.age == 30
        assert reader.is_student is False
        assert reader.grades == [85, 90, 78]
        assert reader.address == {"street": "123 Main St", "city": "Anytown", "zipcode": "12345"}
        assert len(reader.contacts) == 2
        assert reader.contacts[0].type == "email"
        assert reader.contacts[0].value == "john.doe@example.com"
        assert reader.contacts[1].type == "phone"
        assert reader.contacts[1].value == "123-456-7890"

        # Modify and write back to a new YAML file
        reader.name = "Jane Smith"
        reader.age = 25
        reader.address.city = "New City"
        reader.new_attribute = "New Value"
        reader.write("test_modified.yaml")
        print("YAML: Modified file written: test_modified.yaml")

        # Re-initialize another instance and check modified attributes
        modified_reader = YAMLReader("test_modified.yaml")
        assert modified_reader.name == "Jane Smith"
        assert modified_reader.age == 25
        assert modified_reader.is_student is False
        assert modified_reader.grades == [85, 90, 78]
        assert modified_reader.address == {"street": "123 Main St", "city": "New City", "zipcode": "12345"}
        assert len(modified_reader.contacts) == 2
        assert modified_reader.contacts[0].type == "email"
        assert modified_reader.contacts[0].value == "john.doe@example.com"  # Unchanged in modified file
        assert modified_reader.contacts[1].type == "phone"
        assert modified_reader.contacts[1].value == "123-456-7890"  # Unchanged in modified file
        assert modified_reader.new_attribute == "New Value"

        print("YAML: All tests passed successfully!")

    except AssertionError as e:
        print(f"YAML: AssertionError: {e}")
    except Exception as e:
        print(f"YAML: Error occurred during testing: {e}")
    finally:
        import os

        # Clean up: delete temporary files created during testing
        if os.path.exists(yaml_file):
            os.remove(yaml_file)
        if os.path.exists("test_modified.yaml"):
            os.remove("test_modified.yaml")


# Run the test cases
if __name__ == "__main__":
    test_yaml_reader()
