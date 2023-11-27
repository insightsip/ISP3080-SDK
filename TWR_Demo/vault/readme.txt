To generate firmware update packages intall first nrfutil tool here and use the following commands.

For Tag firmware:
.\nrfutil.exe pkg generate --application ..\application\ses\Output\Release\Exe\TWR_Tag.hex --application-version-string "1.0.0" --hw-version 52 --sd-req 0x102 --key-file .\priv.pem TWR_Tag.zip

For Anchorfirmware:
.\nrfutil.exe pkg generate --application ..\application\ses\Output\Release\Exe\TWR_Anchor.hex --application-version-string "1.0.0" --hw-version 52 --sd-req 0x102 --key-file .\priv.pem TWR_Anchor.zip

The private keys (priv.pem) is provided for developement purpose. In production the customer has to generate new set of keys.