import fitz
doc = fitz.open(r'C:\Users\Administrator\Desktop\chendian\733a60ac454e4a959c5f9e4dbb747e4b.pdf')
for i, page in enumerate(doc):
    print(f"=== PAGE {i+1} ===")
    print(page.get_text())
