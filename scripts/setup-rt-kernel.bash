#!/bin/bash

set -e

CURRENT_KERNEL=$(uname -r)
CURRENT_KERNEL_VERSION=$(echo "$CURRENT_KERNEL" | sed -E 's/^([0-9]+\.[0-9]+).*/\1/')
echo "Current kernel version: $CURRENT_KERNEL_VERSION"

RT_BASE_URL="https://cdn.kernel.org/pub/linux/kernel/projects/rt"

echo "Fetching available RT kernel versions..."
RT_VERSIONS_HTML=$(curl -s "$RT_BASE_URL/")

MAJOR_MINOR_VERSIONS=()
while IFS= read -r line; do
    if echo "$line" | grep -qE 'href="[0-9]+\.[0-9]+/"'; then
        VERSION=$(echo "$line" | sed -E 's/.*href="([0-9]+\.[0-9]+)\/".*/\1/')
        MAJOR=$(echo "$VERSION" | cut -d. -f1)
        MINOR=$(echo "$VERSION" | cut -d. -f2)
        CURRENT_MAJOR=$(echo "$CURRENT_KERNEL_VERSION" | cut -d. -f1)
        CURRENT_MINOR=$(echo "$CURRENT_KERNEL_VERSION" | cut -d. -f2)

        if [[ "$MAJOR" -gt "$CURRENT_MAJOR" ]] || \
           [[ "$MAJOR" -eq "$CURRENT_MAJOR" && "$MINOR" -ge "$CURRENT_MINOR" ]]; then
            MAJOR_MINOR_VERSIONS+=("$VERSION")
        fi
    fi
done <<< "$RT_VERSIONS_HTML"

AVAILABLE_VERSIONS=()
for MAJOR_MINOR in "${MAJOR_MINOR_VERSIONS[@]}"; do
    SUB_VERSIONS_HTML=$(curl -s "$RT_BASE_URL/$MAJOR_MINOR/" 2>/dev/null || echo "")

    HAS_SUB_VERSIONS=false
    SUB_VERSIONS=()
    while IFS= read -r line; do
        if echo "$line" | grep -qE 'href="[0-9]+\.[0-9]+\.[0-9]+/"'; then
            SUB_VER=$(echo "$line" | sed -E 's/.*href="([0-9]+\.[0-9]+\.[0-9]+)\/".*/\1/')
            SUB_VERSIONS+=("$SUB_VER")
            HAS_SUB_VERSIONS=true
        fi
    done <<< "$SUB_VERSIONS_HTML"

    if [[ "$HAS_SUB_VERSIONS" == "true" ]]; then
        for SUB in "${SUB_VERSIONS[@]}"; do
            AVAILABLE_VERSIONS+=("$SUB")
        done
    else
        AVAILABLE_VERSIONS+=("$MAJOR_MINOR")
    fi
done

IFS=$'\n' AVAILABLE_VERSIONS=($(sort -V <<< "${AVAILABLE_VERSIONS[*]}"))
IFS=$'\n'

if [[ ${#AVAILABLE_VERSIONS[@]} -eq 0 ]]; then
    echo "No RT kernel versions found newer than $CURRENT_KERNEL_VERSION"
    exit 1
fi

echo "Checking for RT patch and headers availability..."
echo ""

VERSIONS_WITH_PATCHES=()
VERSIONS_WITHOUT_HEADERS=()
VERSIONS_FULL_VERSION=()
for VERSION in "${AVAILABLE_VERSIONS[@]}"; do
    RT_PATCH_HTML=$(curl -s "$RT_BASE_URL/$VERSION/" 2>/dev/null || echo "")
    if echo "$RT_PATCH_HTML" | grep -qE 'href="patch-[0-9]+\.[0-9]+(\.[0-9]+)?-rt[0-9]+\.patch\.xz"'; then
        VERSIONS_WITH_PATCHES+=("$VERSION")

        HAS_HEADERS=false
        FULL_VER=""
        MAJOR_MINOR=$(echo "$VERSION" | sed -E 's/([0-9]+\.[0-9]+).*/\1/')

        GENERIC_HDR=$(apt-cache search "linux-headers-${MAJOR_MINOR}" | grep generic | head -1 || true)
        if [[ -n "$GENERIC_HDR" ]]; then
            HAS_HEADERS=true
            FULL_VER=$(echo "$GENERIC_HDR" | awk '{print $1}' | sed -E 's/linux-headers-([0-9]+\.[0-9]+\.[0-9]+-[0-9]+).*/\1/')
        fi

        VERSIONS_FULL_VERSION+=("$FULL_VER")

        if [[ "$HAS_HEADERS" == "false" ]]; then
            VERSIONS_WITHOUT_HEADERS+=("$VERSION")
        fi
    fi
done

if [[ ${#VERSIONS_WITH_PATCHES[@]} -eq 0 ]]; then
    echo "No RT patches found for any version"
    exit 1
fi

echo "Available RT kernel versions:"
echo "  (*) = linux-headers package available"
echo "  (!) = linux-headers NOT available (may need manual installation)"
echo ""
for i in "${!VERSIONS_WITH_PATCHES[@]}"; do
    VERSION="${VERSIONS_WITH_PATCHES[$i]}"
    FULL_VER="${VERSIONS_FULL_VERSION[$i]}"
    if [[ " ${VERSIONS_WITHOUT_HEADERS[@]} " =~ " ${VERSION} " ]]; then
        if [[ -n "$FULL_VER" ]]; then
            echo "$((i+1))) ${VERSION} (${FULL_VER}) (!)"
        else
            echo "$((i+1))) ${VERSION} (!)"
        fi
    else
        if [[ -n "$FULL_VER" ]]; then
            echo "$((i+1))) ${VERSION} (${FULL_VER}) (*)"
        else
            echo "$((i+1))) ${VERSION} (*)"
        fi
    fi
done
echo ""

read -p "Select kernel version (1-${#VERSIONS_WITH_PATCHES[@]}): " VERSION_CHOICE

while true; do
    INDEX=$((VERSION_CHOICE - 1))
    if [[ "$INDEX" -ge 0 && "$INDEX" -lt ${#VERSIONS_WITH_PATCHES[@]} ]]; then
        SELECTED_VERSION="${VERSIONS_WITH_PATCHES[$INDEX]}"
        SELECTED_FULL_VERSION="${VERSIONS_FULL_VERSION[$INDEX]}"
        break
    else
        read -p "Invalid choice. Please enter a number between 1 and ${#VERSIONS_WITH_PATCHES[@]}: " VERSION_CHOICE
    fi
done

echo ""
echo "Selected RT kernel version: $SELECTED_VERSION ($SELECTED_FULL_VERSION)"

echo ""
echo "=== Checking available headers and finding matching RT patch ==="

MAJOR_MINOR=$(echo "$SELECTED_VERSION" | cut -d. -f1,2)

FULL_KERNEL_VERSION=""
SELECTED_PATCH=""
RT_PATCH_VERSION=""
HEADERS_FULL_VERSION=""

if [[ -n "$SELECTED_FULL_VERSION" ]]; then
    FULL_KERNEL_VERSION="$SELECTED_FULL_VERSION"
    HEADERS_FULL_VERSION="$SELECTED_FULL_VERSION"
    MAJOR_MINOR_PATCH=$(echo "$FULL_KERNEL_VERSION" | sed -E 's/([0-9]+\.[0-9]+)\.[0-9]+-[0-9]+/\1/')

    echo "Using headers: $FULL_KERNEL_VERSION"

    RT_PATCH_HTML=$(curl -s "$RT_BASE_URL/$MAJOR_MINOR_PATCH/" 2>/dev/null || echo "")

    PATCH_FOUND=false
    while IFS= read -r line; do
        if echo "$line" | grep -qE "href=\"patch-${MAJOR_MINOR_PATCH}\.([0-9]+)-rt[0-9]+\.patch\.xz\""; then
            PATCH=$(echo "$line" | sed -E 's/.*href="(patch-[^"]+\.patch\.xz)".*/\1/')
            RT_NUM=$(echo "$PATCH" | sed -E 's/.*-rt([0-9]+)\.patch\.xz/\1/')
            SELECTED_PATCH="$PATCH"
            RT_PATCH_VERSION="$RT_NUM"
            FULL_KERNEL_VERSION=$(echo "$SELECTED_PATCH" | sed -E 's/patch-([0-9]+\.[0-9]+(\.[0-9]+)?)-rt[0-9]+\.patch\.xz/\1/')
            PATCH_FOUND=true
            break
        fi
    done <<< "$RT_PATCH_HTML"

    if [[ "$PATCH_FOUND" == "false" ]]; then
        RT_PATCH_HTML=$(curl -s "$RT_BASE_URL/$SELECTED_VERSION/" 2>/dev/null || echo "")
        while IFS= read -r line; do
            if echo "$line" | grep -qE 'href="patch-[0-9]+\.[0-9]+(\.[0-9]+)?-rt[0-9]+\.patch\.xz"'; then
                PATCH=$(echo "$line" | sed -E 's/.*href="(patch-[^"]+\.patch\.xz)".*/\1/')
                RT_NUM=$(echo "$PATCH" | sed -E 's/.*-rt([0-9]+)\.patch\.xz/\1/')
                SELECTED_PATCH="$PATCH"
                RT_PATCH_VERSION="$RT_NUM"
                FULL_KERNEL_VERSION=$(echo "$SELECTED_PATCH" | sed -E 's/patch-([0-9]+\.[0-9]+(\.[0-9]+)?)-rt[0-9]+\.patch\.xz/\1/')
                break
            fi
        done <<< "$RT_PATCH_HTML"
    fi
else
    RT_PATCH_HTML=$(curl -s "$RT_BASE_URL/$SELECTED_VERSION/")
    while IFS= read -r line; do
        if echo "$line" | grep -qE 'href="patch-[0-9]+\.[0-9]+(\.[0-9]+)?-rt[0-9]+\.patch\.xz"'; then
            PATCH=$(echo "$line" | sed -E 's/.*href="(patch-[^"]+\.patch\.xz)".*/\1/')
            RT_NUM=$(echo "$PATCH" | sed -E 's/.*-rt([0-9]+)\.patch\.xz/\1/')
            SELECTED_PATCH="$PATCH"
            RT_PATCH_VERSION="$RT_NUM"
            FULL_KERNEL_VERSION=$(echo "$SELECTED_PATCH" | sed -E 's/patch-([0-9]+\.[0-9]+(\.[0-9]+)?)-rt[0-9]+\.patch\.xz/\1/')
            break
        fi
    done <<< "$RT_PATCH_HTML"
fi

if [[ -z "$SELECTED_PATCH" ]]; then
    echo "No RT patch found for version $SELECTED_VERSION"
    exit 1
fi

if [[ -n "$SELECTED_FULL_VERSION" && -z "$PATCH_FOUND" ]]; then
    echo "Note: Using RT patch $SELECTED_PATCH but headers are for $SELECTED_FULL_VERSION"
    KERNEL_FROM_HEADERS="$SELECTED_FULL_VERSION"
    FULL_KERNEL_VERSION=$(echo "$SELECTED_PATCH" | sed -E 's/patch-([0-9]+\.[0-9]+(\.[0-9]+)?)-rt[0-9]+\.patch\.xz/\1/')
    echo "Building kernel $FULL_KERNEL_VERSION with headers $KERNEL_FROM_HEADERS"
fi

echo "Using RT patch: $SELECTED_PATCH (kernel $FULL_KERNEL_VERSION)"

if [[ -n "$HEADERS_FULL_VERSION" ]]; then
    INSTALL_DIR="$HOME/rt_kernels/${HEADERS_FULL_VERSION}-rt${RT_PATCH_VERSION}"
else
    INSTALL_DIR="$HOME/rt_kernels/${FULL_KERNEL_VERSION}-rt${RT_PATCH_VERSION}"
fi

if [[ -d "$INSTALL_DIR" ]]; then
    echo "Removing existing directory: $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

KERNEL_SOURCE_VERSION="$FULL_KERNEL_VERSION"
KERNEL_VERSION="$FULL_KERNEL_VERSION"
KERNEL_MAJOR_VERSION=$(echo "$KERNEL_VERSION" | cut -d. -f1)

PATCH_DIR="$MAJOR_MINOR_PATCH"
if [[ -z "$PATCH_DIR" ]]; then
    PATCH_DIR="$SELECTED_VERSION"
fi

echo "Downloading to: $INSTALL_DIR"

wget -q "$RT_BASE_URL/$PATCH_DIR/$SELECTED_PATCH" || echo "Warning: Failed to download RT patch"
wget -q "$RT_BASE_URL/$PATCH_DIR/${SELECTED_PATCH%.xz}.sign" || echo "Warning: Failed to download RT patch signature"

wget -q "https://www.kernel.org/pub/linux/kernel/v$KERNEL_MAJOR_VERSION.x/linux-$KERNEL_SOURCE_VERSION.tar.xz" || echo "Warning: Failed to download kernel"
wget -q "https://www.kernel.org/pub/linux/kernel/v$KERNEL_MAJOR_VERSION.x/linux-$KERNEL_SOURCE_VERSION.tar.sign" || echo "Warning: Failed to download kernel signature"

echo "Download complete!"
echo ""
echo "Summary:"
echo "  Kernel version: $KERNEL_VERSION"
echo "  RT patch: $RT_PATCH_VERSION"
echo "  Install directory: $INSTALL_DIR"

echo ""
echo "=== Extracting and verifying files ==="

echo "Extracting kernel source..."
xz -dk "linux-$KERNEL_VERSION.tar.xz"

if [[ -n "$SELECTED_PATCH" ]]; then
    echo "Extracting RT patch..."
    xz -dk "$SELECTED_PATCH"
fi

echo ""
echo "Verifying kernel source signature..."

KERNEL_SIGN_KEY=$(gpg --armor --extract-keys "linux-$KERNEL_VERSION.tar.sign" 2>/dev/null | grep -E '^.*<.*@.*\..*>.*$' | head -1 || true)
if [[ -z "$KERNEL_SIGN_KEY" ]]; then
    KERNEL_SIGN_KEY=$(gpg --dry-run --import "linux-$KERNEL_VERSION.tar.sign" 2>&1 | grep -E 'keyring|pubring' | head -1 || true)
fi

echo "  Fetching kernel signing keys..."
gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org 2>/dev/null || true

echo "  Verifying linux-$KERNEL_VERSION.tar.sign..."
if gpg2 --verify "linux-$KERNEL_VERSION.tar.sign" "linux-$KERNEL_VERSION.tar" 2>&1; then
    echo "  Kernel source signature: OK"
else
    echo "  Warning: Kernel signature verification failed, but continuing..."
fi

if [[ -n "$SELECTED_PATCH" ]]; then
    echo ""
    echo "Verifying RT patch signature..."

    RT_PATCH_KEY=$(gpg --armor --extract-keys "patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign" 2>/dev/null | grep -E '^.*<.*@.*\..*>.*$' | head -1 || true)

    echo "  Fetching RT patch signing key..."
    gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys AD85102A6BE1CDFE9BCA84F36CEF3D27CA5B141E 2>/dev/null || true
    gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 57892E705233051337F6FDD105641F175712FA5B 2>/dev/null || true

    if gpg2 --verify "patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign" "patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch" 2>&1; then
        echo "  RT patch signature: OK"
    else
        echo "  Warning: RT patch signature verification failed, but continuing..."
    fi
fi

echo ""
echo "Extraction and verification complete!"

echo ""
echo "=== Checking for NVIDIA GPU and drivers ==="

NVIDIA_GPU=$(lspci 2>/dev/null | grep -i "vga\|3d\|display" | grep -i nvidia || true)
if [[ -z "$NVIDIA_GPU" ]]; then
    NVIDIA_GPU=$(nvidia-smi 2>/dev/null || true)
fi

NVIDIA_WAS_INSTALLED=false
NVIDIA_DRIVER_VERSION=""

if [[ -n "$NVIDIA_GPU" ]]; then
    echo "NVIDIA GPU detected:"
    echo "$NVIDIA_GPU" | head -5

    echo ""
    echo "Checking for installed NVIDIA driver..."
    NVIDIA_INSTALLED=$(dpkg -l | grep -E "^ii.*nvidia-driver" | grep -v "^ii.*nvidia-settings" | head -1 || true)

    if [[ -n "$NVIDIA_INSTALLED" ]]; then
        NVIDIA_WAS_INSTALLED=true
        NVIDIA_DRIVER_VERSION=$(echo "$NVIDIA_INSTALLED" | grep -oP '\d+' | head -1 || true)
        echo "NVIDIA driver found (version $NVIDIA_DRIVER_VERSION):"
        echo "$NVIDIA_INSTALLED"

        echo ""
        echo "Removing NVIDIA drivers to avoid conflicts during kernel compilation..."
        sudo apt remove -y nvidia-* || true
        sudo apt autoremove -y || true
        echo "NVIDIA drivers removed. Will reinstall after kernel installation."
    else
        echo "No NVIDIA driver currently installed"
    fi
else
    echo "No NVIDIA GPU detected"
fi

echo ""
echo "=== Unpacking kernel source ==="
tar xf "linux-$KERNEL_VERSION.tar"
cd "linux-$KERNEL_VERSION"

if [[ -n "$SELECTED_PATCH" ]]; then
    echo ""
    echo "=== Applying RT patch ==="
    PATCH_FILE=$(ls ../patch-${SELECTED_VERSION}*.patch 2>/dev/null | head -1 || ls ../patch-*.patch 2>/dev/null | head -1)
    if [[ -n "$PATCH_FILE" ]]; then
        cat "$PATCH_FILE" | patch -p1
    fi
fi

echo ""
echo "=== Installing linux-headers ==="

MAJOR_MINOR=$(echo "$KERNEL_VERSION" | cut -d. -f1,2)

GENERIC_HEADERS=$(apt-cache search "linux-headers-${MAJOR_MINOR}" | grep generic | head -1)
if [[ -n "$GENERIC_HEADERS" ]]; then
    HEADERS_PKG=$(echo "$GENERIC_HEADERS" | awk '{print $1}')
    echo "Installing $HEADERS_PKG..."
    sudo apt install -y "$HEADERS_PKG"
else
    echo "Warning: No generic headers package found for $KERNEL_VERSION"
    echo "Searching for available headers for $MAJOR_MINOR..."
    apt-cache search "linux-headers-${MAJOR_MINOR}" | head -5 || true
fi

echo ""
echo "=== Configuring kernel ==="
make defconfig

./scripts/config --set-str SYSTEM_TRUSTED_KEYS ""
./scripts/config --set-str SYSTEM_REVOCATION_KEYS ""
./scripts/config --set-str MODULE_SIG_KEY "certs/signing_key.pem"
./scripts/config --enable EXPERT
./scripts/config --enable PREEMPT_RT

make olddefconfig

if grep -q "^CONFIG_PREEMPT_RT=y" .config; then
    echo "PREEMPT_RT enabled successfully"
else
    echo "ERROR: PREEMPT_RT not enabled!"
    exit 1
fi

echo ""
echo "=== Compiling kernel ==="
make -j$(nproc)

echo ""
echo "=== Compiling kernel ==="
make -j$(nproc)

echo ""
echo "Compilation complete!"
echo "Kernel image: arch/x86/boot/bzImage"

echo ""
echo "=== Installing kernel ==="
sudo IGNORE_PREEMPT_RT_PRESENCE=1 make modules_install
sudo IGNORE_PREEMPT_RT_PRESENCE=1 make install
sudo IGNORE_PREEMPT_RT_PRESENCE=1 update-grub

echo ""
echo "=== Verifying installation ==="

if [[ -n "$HEADERS_FULL_VERSION" ]]; then
    INSTALLED_VERSION_BASE=$(echo "$HEADERS_FULL_VERSION" | sed -E 's/-[0-9]+$//')
    INSTALLED_VERSION="$INSTALLED_VERSION_BASE-rt$RT_PATCH_VERSION"
elif [[ -n "$FULL_KERNEL_VERSION" ]]; then
    INSTALLED_VERSION_BASE=$(echo "$FULL_KERNEL_VERSION" | sed -E 's/-[0-9]+$//')
    INSTALLED_VERSION="$INSTALLED_VERSION_BASE-rt$RT_PATCH_VERSION"
else
    INSTALLED_VERSION="$KERNEL_VERSION-rt$RT_PATCH_VERSION"
fi

echo "Installed version: $INSTALLED_VERSION"

echo ""
echo "Checking /boot for installed kernel..."
if ls /boot/vmlinuz-* 2>/dev/null | grep -q "$INSTALLED_VERSION"; then
    echo "  Kernel image installed: OK"
    ls -la /boot/vmlinuz-* | grep "$INSTALLED_VERSION"
else
    echo "  Warning: Kernel image not found in /boot"
    ls -la /boot/vmlinuz-*
fi

echo ""
echo "Checking /boot for initrd..."
if ls /boot/initrd.img-* 2>/dev/null | grep -q "$INSTALLED_VERSION"; then
    echo "  Initrd installed: OK"
    ls -la /boot/initrd.img-* | grep "$INSTALLED_VERSION"
else
    echo "  Warning: Initrd not found in /boot"
fi

echo ""
echo "Checking /lib/modules for modules..."
if ls /lib/modules/ | grep -q "$INSTALLED_VERSION"; then
    echo "  Modules installed: OK"
    ls -la /lib/modules/ | grep "$INSTALLED_VERSION"
else
    echo "  Warning: Modules not found in /lib/modules"
    ls -la /lib/modules/
fi

echo ""
echo "Checking GRUB menu..."

# Try multiple version patterns to find the GRUB entry
GRUB_FOUND=false
KERNEL_BASE_VERSION=$(echo "$KERNEL_VERSION" | sed -E 's/([0-9]+\.[0-9]+\.[0-9]+).*/\1/')
RT_VERSION_PATTERN="${KERNEL_BASE_VERSION}-rt${RT_PATCH_VERSION}"

# First try exact match
if sudo grep -q "$INSTALLED_VERSION" /boot/grub/grub.cfg 2>/dev/null; then
    GRUB_FOUND=true
    FOUND_VERSION="$INSTALLED_VERSION"
# Then try base kernel + rt pattern (e.g., 6.17.5-rt7)
elif sudo grep -q "$RT_VERSION_PATTERN" /boot/grub/grub.cfg 2>/dev/null; then
    GRUB_FOUND=true
    FOUND_VERSION="$RT_VERSION_PATTERN"
# Finally try just checking for rt patch number
elif sudo grep -q "rt${RT_PATCH_VERSION}" /boot/grub/grub.cfg 2>/dev/null; then
    GRUB_FOUND=true
    FOUND_VERSION=$(sudo grep "menuentry.*rt${RT_PATCH_VERSION}" /boot/grub/grub.cfg | head -1 | sed -E "s/.*Linux ([0-9]+\.[0-9]+\.[0-9]+-rt[0-9]+).*/\1/")
fi

if [[ "$GRUB_FOUND" == "true" ]]; then
    echo "  GRUB entry found: OK"
    echo "  Kernel version in GRUB: $FOUND_VERSION"
    sudo grep "menuentry.*$FOUND_VERSION" /boot/grub/grub.cfg | head -3
else
    echo "  Warning: GRUB entry not found for $INSTALLED_VERSION or $RT_VERSION_PATTERN"
    echo "  Current GRUB entries:"
    sudo grep "menuentry.*Linux" /boot/grub/grub.cfg | head -10
fi

echo ""
echo "=== Installation complete ==="
echo "You can now reboot and select '$INSTALLED_VERSION' from GRUB menu"

echo ""
echo "=== Checking for linux-headers ==="

if [[ " ${VERSIONS_WITHOUT_HEADERS[@]} " =~ " ${SELECTED_VERSION} " ]]; then
    echo "linux-headers-$SELECTED_VERSION not available in apt"
    echo "Checking for alternative headers package..."

    MAJOR_MINOR=$(echo "$SELECTED_VERSION" | sed -E 's/([0-9]+\.[0-9]+).*/\1/')
    if apt-cache show "linux-headers-$MAJOR_MINOR" >/dev/null 2>&1; then
        echo "Using linux-headers-$MAJOR_MINOR instead"
    else
        echo "Warning: No headers package found for $SELECTED_VERSION"
        echo "You may need to install headers manually or use a different kernel version"
    fi
else
    echo "linux-headers-$SELECTED_VERSION available"
fi

echo ""
echo "=== Installing NVIDIA drivers ==="

if [[ -n "$NVIDIA_GPU" ]]; then
    if [[ "$NVIDIA_WAS_INSTALLED" == "true" && -n "$NVIDIA_DRIVER_VERSION" ]]; then
        echo "Reinstalling NVIDIA driver (version $NVIDIA_DRIVER_VERSION)..."

        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt update
        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt install -y "nvidia-driver-$NVIDIA_DRIVER_VERSION" || \
        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt install -y nvidia-driver

        echo ""
        echo "Installing NVIDIA DKMS module for RT kernel..."
        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt install -y "nvidia-dkms-$NVIDIA_DRIVER_VERSION" 2>/dev/null || \
        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt install -y nvidia-dkms

    else
        echo "Installing NVIDIA driver for the first time..."

        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt update
        IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt install -y nvidia-driver

        NVIDIA_NEW_VERSION=$(dpkg -l | grep -E "^ii.*nvidia-driver" | grep -v "^ii.*nvidia-settings" | head -1 || true)
        NVIDIA_DRIVER_VERSION=$(echo "$NVIDIA_NEW_VERSION" | grep -oP '\d+' | head -1 || true)

        if [[ -n "$NVIDIA_DRIVER_VERSION" ]]; then
            echo ""
            echo "Installing NVIDIA DKMS module for RT kernel..."
            IGNORE_PREEMPT_RT_PRESENCE=1 sudo apt install -y nvidia-dkms
        fi
    fi

    echo ""
    echo "Verifying NVIDIA module for RT kernel..."
    NVIDIA_MODULE_PATH="/lib/modules/$INSTALLED_VERSION/updates/dkms/nvidia.ko"

    if [[ -f "$NVIDIA_MODULE_PATH" ]]; then
        echo "NVIDIA module built for RT kernel: OK"
        ls -lh "$NVIDIA_MODULE_PATH"
    else
        echo "Warning: NVIDIA module not found at $NVIDIA_MODULE_PATH"
        echo "The module may be built on next reboot, or you may need to run:"
        echo "  sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg-reconfigure nvidia-dkms-$NVIDIA_DRIVER_VERSION"
    fi
else
    echo "No NVIDIA GPU detected, skipping NVIDIA driver installation"
fi

echo ""
echo "=== Done ==="
