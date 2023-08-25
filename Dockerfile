FROM ubuntu:20.04

WORKDIR /app

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Install Java 11 for e2 studio
RUN apt-get update && apt-get -y --fix-missing --no-install-recommends install \
    openjdk-11-jre xz-utils ca-certificates tar xzip gzip bzip2 zip \
    unzip rsync wget libpython2.7 curl xmlstarlet git git-lfs python3-pip && \
    pip3 install cryptography==3.3.2 intelhex click cbor && \
    wget --progress=bar:force:noscroll https://github.com/mikefarah/yq/releases/download/v4.9.3/yq_linux_amd64 -O /usr/bin/yq && \
    chmod +x /usr/bin/yq && \
    wget --progress=bar:force:noscroll https://github.com/stedolan/jq/releases/download/jq-1.6/jq-linux64 -O /usr/bin/jq && \
    chmod +x /usr/bin/jq

# Variables that should not be changing per release of FSP
ENV ENV_E2STUDIO_DEFAULT_WS=/app/workspace
ENV ENV_E2STUDIO_INSTALLER=e2studio_installer-2023-04_linux_host.run

# Download and install e2 studio.
# Remove -clean and -removeClean lines from e2studio.ini. These options are only needed for ugprades to install language packs.
# Use --appimage-extract-and-run to avoid issue when using docker. See here: https://github.com/AppImage/AppImageKit/issues/841
RUN mkdir -p /tmp/fsp && \
    mkdir -p /opt/e2studio && \
    wget --progress=bar:force:noscroll https://cdn.edgeimpulse.com/build-system/${ENV_E2STUDIO_INSTALLER} -O /tmp/fsp/${ENV_E2STUDIO_INSTALLER} && \
    chmod +x /tmp/fsp/${ENV_E2STUDIO_INSTALLER} && \
    /tmp/fsp/${ENV_E2STUDIO_INSTALLER} --appimage-extract-and-run --launcher.suppressErrors -noSplash -install.silent -install.Declipse.p2.default.renesas.skipDriverInstall=true -install.Declipse.p2.default.createLauncherShortcuts=false -install.Declipse.p2.default.defaultInstallLoc=/opt/e2studio && \
    sed -i '/-clean/d' /opt/e2studio/eclipse/e2studio.ini && \
    sed -i '/-removeClean/d' /opt/e2studio/eclipse/e2studio.ini && \
    ln -s /opt/e2studio/eclipse/e2studio /usr/bin/e2studio && \
    rm -rf /tmp/fsp

ENV ENV_GCC=gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
# Can find this by calling arm-none-eabi-gcc --version
ENV ENV_GCC_VERSION_STRING=10.3.1.20210824

# Download and install GCC
RUN mkdir -p /tmp/fsp && \
    wget --progress=bar:force:noscroll https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/10.3-2021.10/${ENV_GCC} -O /tmp/fsp/${ENV_GCC} && \
    tar -xf /tmp/fsp/${ENV_GCC} --directory /opt && \
    rm -rf /tmp/fsp

# Add toolchainStore.xml so e2 studio will recognize GCC
ENV ENV_E2STUDIO_TOOLSTORE=/opt/e2studio/eclipse/configuration/com.renesas.cdt.core/toolchainStore.xml
RUN echo '<?xml version="1.0" encoding="UTF-8" standalone="no"?><toolchainStore version="1"> \
    <toolchain enabled="true" name="GNU ARM Embedded" path="" typeId="gcc-arm-embedded" version=""/> \
    </toolchainStore>' > ${ENV_E2STUDIO_TOOLSTORE}

# Update toolchainStore.xml so toolchain(s) are recognized
RUN xmlstarlet edit --inplace --update "/toolchainStore/toolchain/@path" --value "/opt/${ENV_GCC%-x86_64-linux.tar.bz2}" ${ENV_E2STUDIO_TOOLSTORE} && \
    xmlstarlet edit --inplace --update "/toolchainStore/toolchain/@version" --value "${ENV_GCC_VERSION_STRING}" ${ENV_E2STUDIO_TOOLSTORE}

# copy (create) script used on later stages
RUN mkdir -p /app/scripts && \
    echo 'loadModule("/RA/ProjectGen")' > /app/scripts/minimum_ease.py

# download and install FSP
# Remove any packs that may have been installed through platform installer. We will install based on ENV_FSP_RELEASE.
# Generate .eclipse/com.renesas.platform_<random> directory so we can install FSP to it.
# The || true is on the end because this call will currently fail
ENV ENV_FSP_PACK_VERSION=v4.2.0
RUN rm -rf /opt/e2studio/internal && \
    rm -rf /root/.eclipse && \
    /usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.ease.runScript -data ${ENV_E2STUDIO_DEFAULT_WS} -script /app/scripts/minimum_ease.py -clean || true && \
    TMP_E2STUDIO_SUPPORT_AREA=$(find /root/.eclipse -mindepth 1 -maxdepth 1 -regextype sed -regex ".*/com\.renesas\.platform_[0-9]*" -type d) && \
    mkdir -p /tmp/fsp && \
    wget --progress=bar:force:noscroll https://github.com/renesas/fsp/releases/download/${ENV_FSP_PACK_VERSION}/FSP_Packs_${ENV_FSP_PACK_VERSION}.zip -O /tmp/fsp/FSP_Packs_${ENV_FSP_PACK_VERSION}.zip && \
    unzip -q -d ${TMP_E2STUDIO_SUPPORT_AREA} /tmp/fsp/FSP_Packs_${ENV_FSP_PACK_VERSION}.zip && \
    /usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.ease.runScript -data ${ENV_E2STUDIO_DEFAULT_WS} -script /app/scripts/minimum_ease.py || true && \
    rm -rf /tmp/fsp

# recreate workspace
RUN rm -rf ${ENV_E2STUDIO_DEFAULT_WS} && \
    mkdir -p ${ENV_E2STUDIO_DEFAULT_WS}/.metadata/.plugins/org.eclipse.core.runtime/.settings/ && \
    /usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data ${ENV_E2STUDIO_DEFAULT_WS} -cleanBuild all > /tmp/e2studio.log 2>&1


# RUN /usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data ${ENV_E2STUDIO_DEFAULT_WS} -import ${ENV_E2STUDIO_DEFAULT_WS}/firmware-renesas-ck-ra6m5 -cleanBuild firmware-renesas-ck-ra6m5 -debug -consolelog

CMD chmod +x ${ENV_E2STUDIO_DEFAULT_WS}/firmware-renesas-ck-ra6m5/build.sh && ${ENV_E2STUDIO_DEFAULT_WS}/firmware-renesas-ck-ra6m5/build.sh
