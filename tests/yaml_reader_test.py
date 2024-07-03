#!/usr/bin/env python3

import yaml


class YAMLReader:
    """Read YAML files and store the data in an attribute style access object. This only reads at instance creation."""

    def __init__(self, file_path: str = None) -> None:
        """Initialize the YAML Reader using the .yaml file path. Use full path."""
        if file_path is None:
            raise ValueError("No YAML file path provided")

        try:
            with open(file_path, "r") as file:
                print(f"Loading YAML file: {file_path}")
                data = yaml.safe_load(file)
                if data:
                    self._update_attributes(data)
        except Exception as e:
            print(f"Failed to load YAML file: {e}")

    def _update_attributes(self, data):
        """Update the class instance attributes with the data from the YAML file"""
        for key, value in data.items():
            setattr(self, key, value)

    def write(self, file_path=None):
        """Write the instance attributes to a YAML file"""
        if file_path is None:  # If file path is not provided, use the class attribute
            raise ValueError("No YAML file path provided")
        data = {key: value for key, value in self.__dict__.items() if not key.startswith("_")}
        try:
            with open(file_path, "w") as file:
                yaml.safe_dump(data, file)
                print(f"YAML file successfully written to: {file_path}")
        except Exception as e:
            print(f"Failed to write YAML file: {e}")

    def __getattr__(self, name) -> None:
        """If attribute is not found, return None"""
        return None

    def __setattr__(self, name, value) -> None:
        """Set attribute if it is not a private attribute"""
        if name.startswith("_") or name in self.__dict__:
            super().__setattr__(name, value)
        else:
            self.__dict__[name] = value


# Test cases
def test_yaml_reader():
    # Initialize YAMLReader with a sample YAML file
    yaml_file = "test.yaml"
    initial_data = {
        "name": "John Doe",
        "age": 30,
        "is_student": False,
        "grades": [85, 90, 78],
        "address": {"street": "123 Main St", "city": "Anytown", "zipcode": "12345"},
    }

    try:
        with open(yaml_file, "w") as file:
            yaml.safe_dump(initial_data, file)
            print(f"Sample YAML file created: {yaml_file}")

        reader = YAMLReader(yaml_file)

        # Check existing attributes
        assert reader.name == "John Doe"
        assert reader.age == 30
        assert reader.is_student is False
        assert reader.grades == [85, 90, 78]
        assert reader.address == {"street": "123 Main St", "city": "Anytown", "zipcode": "12345"}

        # Check non-existent attributes
        assert reader.non_existent_attribute is None
        assert reader.another_non_existent_attribute is None

        # Modify attributes and write to a new YAML file
        reader.name = "Jane Smith"
        reader.age = 25
        reader.new_attribute = "New Value"
        reader.write("test_modified.yaml")

        # Re-initialize another instance and check modified attributes
        modified_reader = YAMLReader("test_modified.yaml")
        assert modified_reader.name == "Jane Smith"
        assert modified_reader.age == 25
        assert modified_reader.is_student is False
        assert modified_reader.grades == [85, 90, 78]
        assert modified_reader.address == {"street": "123 Main St", "city": "Anytown", "zipcode": "12345"}
        assert modified_reader.new_attribute == "New Value"

        # Check non-existent attributes in modified instance
        assert modified_reader.non_existent_attribute is None
        assert modified_reader.another_non_existent_attribute is None

        print("All tests passed successfully!")

    except AssertionError as e:
        print(f"AssertionError: {e}")
    except Exception as e:
        print(f"Error occurred during testing: {e}")
    finally:
        import os

        if os.path.exists(yaml_file):
            os.remove(yaml_file)
        if os.path.exists("test_modified.yaml"):
            os.remove("test_modified.yaml")


# Run the test cases
if __name__ == "__main__":
    test_yaml_reader()
