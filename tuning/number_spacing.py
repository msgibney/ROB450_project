def four_steps(a, b):
    step = (b - a) / 5
    return [a + step * i for i in range(1, 5)]

while True:
    user_input = input("\nEnter two positive numbers separated by a space (or 'q' to quit): ")

    if user_input.lower() == 'q':
        print("Goodbye!")
        break

    try:
        a_str, b_str = user_input.split()
        a = float(a_str)
        b = float(b_str)

        if a <= 0 or b <= 0:
            raise ValueError

        results = four_steps(a, b)
        for num in results:
            print(f"{num:.2f}")

    except ValueError:
        print("Please enter two valid positive numbers.")