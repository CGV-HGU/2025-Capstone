import os
import sys
import subprocess
import zipfile
import re

REPO_DIR = r"C:\Users\USER\Desktop\캡스톤\V-Lidar-Research"

TARGETS = [
    {
        "name": "IEEE_Access",
        "dir": os.path.join(REPO_DIR, "paper", "workspace", "IEEE_Access"),
        "tex": "main.tex",
        "zip": os.path.join(REPO_DIR, "paper", "workspace", "IEEE_Access_Overleaf.zip"),
    },
    {
        "name": "Elsevier_CEE",
        "dir": os.path.join(REPO_DIR, "paper", "workspace", "Elsevier_CEE"),
        "tex": "main.tex",
        "zip": os.path.join(REPO_DIR, "paper", "workspace", "Elsevier_CEE_Overleaf.zip"),
    },
    {
        "name": "IOP_MST",
        "dir": os.path.join(REPO_DIR, "paper", "workspace", "IOP_MST"),
        "tex": "main.tex",
        "zip": os.path.join(REPO_DIR, "paper", "workspace", "IOP_MST_Overleaf.zip"),
    },
    {
        "name": "manuscript",
        "dir": os.path.join(REPO_DIR, "paper", "manuscript"),
        "tex": "main.tex",
        "zip": os.path.join(REPO_DIR, "paper", "v_lidar_manuscript_overleaf.zip"),
    },
]

def run_cmd(cmd, cwd):
    res = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, shell=True)
    return res.returncode, res.stdout

def compile_target(t):
    name = t["name"]
    cwd = t["dir"]
    if not os.path.exists(cwd):
        print(f"[{name}] Skipping (directory not found: {cwd})")
        return True

    tex_base = os.path.splitext(t["tex"])[0]
    print(f"=== Compiling {name} in {cwd} ===")
    
    # 1. pdflatex
    rc, out = run_cmd(f"pdflatex -interaction=nonstopmode {t['tex']}", cwd)
    if rc != 0:
        print(f"[{name}] pdflatex pass 1 returned non-zero code {rc}")
    
    # 2. bibtex
    rc, out = run_cmd(f"bibtex {tex_base}", cwd)
    if rc != 0:
        print(f"[{name}] bibtex returned non-zero code {rc}")
        print(out[-500:])
        
    # 3. pdflatex pass 2
    rc, out = run_cmd(f"pdflatex -interaction=nonstopmode {t['tex']}", cwd)
    
    # 4. pdflatex pass 3
    rc, out = run_cmd(f"pdflatex -interaction=nonstopmode {t['tex']}", cwd)
    
    # Audit log
    log_file = os.path.join(cwd, f"{tex_base}.log")
    pdf_file = os.path.join(cwd, f"{tex_base}.pdf")
    
    if not os.path.exists(pdf_file):
        print(f"ERROR: {pdf_file} was not generated!")
        return False
        
    pdf_size = os.path.getsize(pdf_file)
    print(f"[{name}] PDF generated: {pdf_size:,} bytes")
    
    with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
        log_content = f.read()
        
    undef_cites = re.findall(r"Citation `(.*?)' on page \d+ undefined", log_content)
    undef_refs = re.findall(r"Reference `(.*?)' on page \d+ undefined", log_content)
    pages = re.findall(r"Output written on \S+ \((\d+) pages", log_content)
    
    page_count = pages[0] if pages else "unknown"
    print(f"[{name}] Pages: {page_count}, Undefined Cites: {len(undef_cites)}, Undefined Refs: {len(undef_refs)}")
    if undef_cites:
        print(f"  Undefined cites: {set(undef_cites)}")
    if undef_refs:
        print(f"  Undefined refs: {set(undef_refs)}")

    if name == "IEEE_Access":
        root_pdf = os.path.join(REPO_DIR, "paper", "IEEE_access.pdf")
        import shutil
        shutil.copy2(pdf_file, root_pdf)
        print(f"[{name}] Copied {pdf_file} -> {root_pdf} ({os.path.getsize(root_pdf):,} bytes)")

    return True

def package_target(t):
    name = t["name"]
    cwd = t["dir"]
    if not os.path.exists(cwd):
        return
    zip_path = t["zip"]
    print(f"=== Packaging {name} to {zip_path} ===")
    
    exclude_exts = {".aux", ".bbl", ".blg", ".log", ".out", ".spl", ".synctex.gz", ".fls", ".fdb_latexmk"}
    
    file_count = 0
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(cwd):
            for file in files:
                ext = os.path.splitext(file)[1].lower()
                if ext in exclude_exts:
                    continue
                full_path = os.path.join(root, file)
                rel_path = os.path.relpath(full_path, cwd)
                zf.write(full_path, rel_path)
                file_count += 1
                
    zip_size = os.path.getsize(zip_path)
    print(f"[{name}] Packaged {file_count} files into {zip_size:,} bytes.")

def main():
    success = True
    for t in TARGETS:
        ok = compile_target(t)
        if not ok:
            success = False
        else:
            package_target(t)
            
    print("\n=== COMPILATION AND PACKAGING SUMMARY ===")
    print(f"Overall status: {'SUCCESS' if success else 'FAILED'}")

if __name__ == "__main__":
    main()
