---
pagetitle: List of Projects delivered in X-CUBE-SBSFU Expansion package
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="../_htmresc/favicon.png" />
---

<center>
# List of Projects delivered in X-CUBE-SBSFU Expansion package
Copyright &copy; 2005 STMicroelectronics\

[![ST logo](../_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>

::: {.row}
::: {.col-sm-12 .col-lg-12}

# Purpose

The table beside details the available sample __Applications__ per __Board__, including information on:

- the __Security mechanism__ involved

- the __Isolation method__ deployed and the subsequent availability of __Runtime services__

- the __SW cryptographic library__ in user

- the selectable __Crypto scheme__


# Projects list

STM32 Board             Application         Security mechanisms                                          Isolation method                Runtime services  SW CryptoLibrary      Crypto Scheme(s)
------------            ------------        --------------------                                        ------------------              ------------------ -----------------     -----------------
32L496GDISCOVERY        2_Images            Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_OSC        Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
B-L475E-IOT01A          2_Images            Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_ExtFlash   Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_KMS        Firewall, MPU, WRP, RDP, DAP, IWDG, Tamper                   Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_OSC        Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
B-L4S5I-IOT01A          2_Images_KMS        Firewall, MPU, WRP, RDP, DAP, IWDG, Tamper                   Firewall                        Yes                MBED                 X509_ECDSA_WITHOUT_ENCRYPT_SHA256
                        2_Images_STSAFE     Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                MBED                 X509_ECDSA_WITHOUT_ENCRYPT_SHA256
NUCLEO-G031K8           1_Image             Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-G071RB           1_Image             Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-G474RE           1_Image             Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-H753ZI           1_Image             Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_OSC        Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-L073RZ           2_Images            Firewall, MPU, WRP, RDP, DAP, IWDG, Tamper                   Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-L152RE           1_Image             MPU, WRP, RDP, DAP, IWDG, Tamper                             MPU                             Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            MPU, WRP, RDP, DAP, IWDG, Tamper                             MPU                             Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-L432KC           1_Image             Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
NUCLEO-L476RG           2_Images            Firewall, PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper            Firewall                        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
P-NUCLEO-WB55.Nucleo    1_Image             MPU, WRP, RDP, DAP, IWDG, Tamper, CKS                        AES keys isolated in CM0        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            MPU, WRP, RDP, DAP, IWDG, Tamper, CKS                        AES keys isolated in CM0        Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_OSC        MPU, WRP, RDP, DAP, IWDG, Tamper, CKS                        AES keys isolated in CM0        Yes                MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
STM32F413H-Discovery    1_Image             MPU, WRP, RDP, DAP, IWDG, Tamper                             MPU                             Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            MPU, WRP, RDP, DAP, IWDG, Tamper                             MPU                             Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_OSC        MPU, WRP, RDP, DAP, IWDG, Tamper                             MPU                             Yes                MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
STM32F769I-Discovery    1_Image             MPU, WRP, RDP, DAP, IWDG                                     MPU                             Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            MPU, WRP, RDP, DAP, IWDG                                     MPU                             Yes                ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_OSC        MPU, WRP, RDP, DAP, IWDG                                     MPU                             Yes                MBED                 ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
STM32H7B3I-DK           1_Image             Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images            Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM
                        2_Images_ExtFlash   Secure user mem., PCROP, MPU, WRP, RDP, DAP, IWDG, Tamper    MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CTR_SHA256
STM32H750B-DK           2_Images_ExtFlash   Secure user mem., PCROP, MPU, RDP, DAP, IWDG, Tamper         MPU then Secure user mem.       No                 ST Crypto            ECCDSA_WITH_AES128_CBC_SHA256, ECCDSA_WITHOUT_ENCRYPT_SHA256, AES128_GCM_AES128_GCM_AES128_GCM


> **Notes:**
>
> *MPU then Secure user mem.:* MPU isolation is used during SBSFU execution, then Secure user mem. is activated when User Application starts.
>
> *AES keys isolated in CM0:* AES keys are isolated by being stored and manipulated by CM0 core only.

:::

:::

<footer class="sticky">
For complete documentation on **Security framework for STM32 series**, visit: [STM32Trust](https://www.st.com/stm32trust)
</footer>