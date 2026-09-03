# 🖥️ 원격 접속 정보 (Remote Access Info)

NetBird 가상 사설망(VPN)을 통해 본 PC에 원격 접속하기 위한 기본 정보입니다.

---

## 📌 접속 계정 정보

| 항목 | 정보 |
| :--- | :--- |
| **NetBird IP** | `100.96.194.210` |
| **FQDN (도메인)** | `cgv-omo-01.nb.hsl.ee` |
| **아이디 (ID)** | `cgv` |
| **비밀번호 (PW)** | `admin` |

> ⚠️ **주의사항**: 접속하려는 외부 PC에서도 **NetBird가 실행(Connected 상태)**되어 있어야 합니다.

---

## 🚀 접속 방법

### 1. SSH 접속 (터미널 / PowerShell / CMD)
```bash
ssh cgv@100.96.194.210
```
* 비밀번호 입력창에 `admin` 입력

---

### 2. VS Code (Visual Studio Code Remote - SSH)
1. VS Code에서 **Remote - SSH** 확장 설치
2. `F1` 키 → `Remote-SSH: Connect to Host...` 선택
3. `cgv@100.96.194.210` 입력 후 연결
4. 비밀번호 `admin` 입력

---

### 3. Windows 원격 데스크톱 (RDP / Windows App / mstsc)
1. `Win + R` 실행창에 `mstsc` 입력 후 실행 (또는 `Windows App` 실행)
2. **컴퓨터(C)**: `100.96.194.210` 입력 후 **연결** 클릭
3. 계정 정보 입력:
   * **사용자 이름**: `cgv`
   * **비밀번호**: `admin`
