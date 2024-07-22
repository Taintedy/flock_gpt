import openai
import re
import json

def load_or_create_json(filename):
    try:
        with open(filename, 'r') as file:
            data = json.load(file)
    except FileNotFoundError:
        data = []
    return data

def save_to_json(filename, data):
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)
        
# def input_not_in_dataset(input, dataset):
#     for item in dataset:
#         if item["input"] == input:
#             return False
#     return True

model = "gpt-3.5-turbo"

if model == "gpt-3.5-turbo":
    json_filename = "sdf_ds_3.5-turbo.json"
if model == "gpt-4-turbo":
    json_filename = "sdf_ds_4.json"

sdf_data = load_or_create_json(json_filename)

OPENAI_API_KEY = ''
openai.api_key = OPENAI_API_KEY

gpt4_input = """Generate 3D meshes based on SDFs (signed distance functions) with this examples:

### Example of full code

```
from sdf import *

f = sphere(1) & box(1.5)

c = cylinder(0.5)
f -= c.orient(X) | c.orient(Y) | c.orient(Z)

f.save('out.stl')
```

### All functional examples:

sphere
```
f = sphere() # unit sphere
f = sphere(2) # specify radius
f = sphere(1, (1, 2, 3)) # translated sphere
```

box
```
f = box(1) # all side lengths = 1
f = box((1, 2, 3)) # different side lengths
f = box(a=(-1, -1, -1), b=(3, 4, 5)) # specified by bounds
```

rounded_box
```
f = rounded_box((1, 2, 3), 0.25)
```

wireframe_box
```
f = wireframe_box((1, 2, 3), 0.05)
```

torus
```
f = torus(1, 0.25)
```

capsule
```
f = capsule(-Z, Z, 0.5)
```

capped_cylinder
```
f = capped_cylinder(-Z, Z, 0.5)
```

rounded_cylinder
```
f = rounded_cylinder(0.5, 0.1, 2)
```

capped_cone
```
f = capped_cone(-Z, Z, 1, 0.5)
```

rounded_cone
```
f = rounded_cone(0.75, 0.25, 2)
```

ellipsoid
```
f = ellipsoid((1, 2, 3))
```

pyramid
```
f = pyramid(1)
```

tetrahedron
```
f = tetrahedron(1)
```

octahedron
```
f = octahedron(1)
```

dodecahedron
```
f = dodecahedron(1)
```

icosahedron
```
f = icosahedron(1)
```

plane
```
f = sphere() & plane()
```

slab
```
f = sphere() & slab(z0=-0.5, z1=0.5, x0=0)
```

cylinder
```
f = sphere() - cylinder(0.5)
```

text
```
FONT = 'ArialRegular.ttf'
TEXT = 'Hello, world!'

w, h = measure_text(FONT, TEXT)

f = text(FONT, TEXT).extrude(0.1)
```

translate
```
f = sphere().translate((0, 0, 2))
```

scale
```
f = sphere().scale(2)
f = sphere().scale((1, 2, 3)) # non-uniform scaling
```

rotate
```
f = capped_cylinder(-Z, Z, 0.5).rotate(pi / 4, X)
```

orient
```
c = capped_cylinder(-Z, Z, 0.25)
f = c.orient(X) | c.orient(Y) | c.orient(Z)
```

union
```
f = a | b
f = union(a, b) # equivalent
```

difference
```
f = a - b
f = difference(a, b) # equivalent
```

intersection
```
f = a & b
f = intersection(a, b) # equivalent
```

smooth_union
```
f = a | b.k(0.25)
f = union(a, b, k=0.25) # equivalent
```

smooth_difference
```
f = a - b.k(0.25)
f = difference(a, b, k=0.25) # equivalent
```

smooth_intersection
```
f = a & b.k(0.25)
f = intersection(a, b, k=0.25) # equivalent
```

repeat
```
f = sphere().repeat(3, (1, 1, 0))
```

circular_array
```
f = capped_cylinder(-Z, Z, 0.5).circular_array(8, 4)
```

blend
```
f = sphere().blend(box())
```

dilate
```
f = example.dilate(0.1)
```

erode
```
f = example.erode(0.1)
```

shell
```
f = sphere().shell(0.05) & plane
(-Z)
```

elongate
```
f = example.elongate((0.25, 0.5, 0.75))
```

twist
```
f = box().twist(pi / 2)
```

bend
```
f = box().bend(1)
```

bend_linear
```
f = capsule(-Z * 2, Z * 2, 0.25).bend_linear(-Z, Z, X, ease.in_out_quad)
```

bend_radial
```
f = box((5, 5, 0.25)).bend_radial(1, 2, -1, ease.in_out_quad)
```

transition_linear
```
f = box().transition_linear(sphere(), e=ease.in_out_quad)
```

transition_radial
```
f = box().transition_radial(sphere(), e=ease.in_out_quad)
```

wrap_around
```
FONT = 'Arial'
TEXT = ' wrap_around ' * 3
w, h = measure_text(FONT, TEXT)
f = text(FONT, TEXT).extrude(0.1).orient(Y).wrap_around(-w / 2, w / 2)
```

extrude
```
f = hexagon(1).extrude(1)
```

extrude_to
```
f = rectangle(2).extrude_to(circle(1), 2, ease.in_out_quad)
```

revolve
```
f = hexagon(1).revolve(3)
```

slice
```
f = example.translate((0, 0, 0.55)).slice().extrude(0.1)
```

Please be careful when constructing complex figures considering the applications of parts relative to each other.

### Task:

Make code to model 'out.stl' of a 
"""

class SDFModel:
    def __init__(self):
        self.f = None

    def save(self, filename='out.stl'):
        if self.f:
            self.f.save(filename)
        else:
            print("No model to save.")

class SDFDialog:
    def __init__(self, system_prompt=gpt4_input):
        self.messages = []
        self.system_prompt = system_prompt

    def add_user_message(self, content):
        self.messages.append({"role": "user", "content": content})

    def add_assistant_message(self, content):
        self.messages.append({"role": "assistant", "content": content})

    def get_last_sdf_code(self, output):
        pattern = r'```(python)?\s*(.*?)```'
        sdf_code_match = re.findall(pattern, output, re.DOTALL)
        if sdf_code_match:
            sdf_code = sdf_code_match[-1][1]  # Берем последнее совпадение и его второй элемент (содержимое)
            return re.sub(r"f\.save\('out\.stl'\)", "", sdf_code)
        else:
            return ""

    def get_next_sdf_code(self, user_input):
        gpt4_input = self.system_prompt + user_input + ". The last line must be f.save('out.stl')."

        response = openai.ChatCompletion.create(
            model=model,
            max_tokens=1000,
            temperature=0.7,
            messages=self.messages + [{"role": "user", "content": user_input},
                                       {"role": "user", "content": gpt4_input}]
        )

        output = response["choices"][0]["message"]["content"]
        self.messages.append({"role": "assistant", "content": output})
        
        return self.get_last_sdf_code(output)

    def clear_dialog(self):
        self.messages = []


