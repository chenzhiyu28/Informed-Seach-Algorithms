def get_names() -> tuple[str, str]:
    name = str(input())
    return tuple(name.split(" "))

print(get_names())