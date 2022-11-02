# Mise à jour du logiciel interne

## Télécharger le firmware

Obtenez le fichier *windbird-firmware-x.x.x.bin* de la dernière version, à l'adresse https://github.com/windbird-sensor/windbird-fox-firmware/releases.

## Connectez votre Windbird

Utilisez n'importe quel câble USB-to-UART de 3,3 V (attention à ne pas utiliser un câble de 5 V).

Nous recommandons le câble **FTDI TTL-232R-RPI** ou *FTDI TTL-232R-3V3*.

Vous pouvez également trouver des câbles chinois très bon marché (basés sur *FT232RL*, *PL2303*, *CP2102*, *CH340G*...), ils fonctionneront aussi.

![FTDI TTL-232R-RPI](https://user-images.githubusercontent.com/1681443/199475597-df15238b-3611-43d0-8b0c-2d804575dbf8.png)

Connectez le câble à la carte. Si vous utilisez une autre marque que FTDI, les couleurs des fils peuvent être différentes.

![wb uart wiring](https://user-images.githubusercontent.com/1681443/199485309-e7c597e3-cab4-4dad-a36c-c2b6f6c09b66.png)

La batterie doit être branchée. Il n'est pas nécessaire de retirer la carte du corps en plastique du Windbird.

Une fois la mise à jour terminée, vous devez déconnecter le câble `TX` de la carte : le GPS ne fonctionnera pas tant que le fil `TX` sera connecté. Les autres fils (`RX` et `GND`) ne causeront aucune perturbation s'ils sont laissés connectés .

## Mise à jour depuis Windows

Téléchargez et extrayez le logiciel TDLoader : https://github.com/windbird-sensor/windbird-fox-firmware/raw/main/usb-flasher/windows/tdloader.zip

Ouvrez le *Device Manager* en utilisant le raccourci fourni.

Dans le gestionnaire de périphériques, identifiez votre câble et notez le numéro de port. Dans cet exemple, le numéro de port est 4. 

![Capture d’écran du 2022-11-02 13-16-02](https://user-images.githubusercontent.com/1681443/199487441-032c4621-0b38-4532-a9c7-2051d5b8fb94.png)

Si votre câble ne s'affiche pas, il se peut que vous deviez installer le pilote du câble. Le pilote pour les câbles FTDI est inclus dans tdloader.zip. Pour les autres marques, veuillez vous référer aux instructions du fabricant.

Lancez le logiciel TDLoader.

Entrez le numéro de port que vous avez noté précédemment.

Sélectionnez *TD1208 EVB* comme *Product*.

Cliquez sur *Browse* et sélectionnez le fichier du firmware.

![Capture d’écran du 2022-11-02 13-21-43](https://user-images.githubusercontent.com/1681443/199488406-7a34f5e2-d1e5-4f06-a38c-b73c3f50e2f5.png)

A cette étape, assurez-vous que votre Windbird est éteint et que la batterie est connectée.

Appuyez ensuite sur *Acquire* pour lancer le processus de mise à niveau.

Lorsque vous lisez *Synchronizing* à l'écran, allumez votre Windbird en appuyant sur son bouton d'alimentation.

Attendez quelques secondes. Le processus de mise à jour va commencer. Ne touchez à rien, ne quittez rien et ne déconnectez rien pendant la mise à jour.

![image1059](https://user-images.githubusercontent.com/1681443/199490009-47fe796f-a90e-4713-8298-b9a75489bc0b.png)

Lorsque vous lisez *Upgrade OK*, le Windbird est prêt et va redémarrer. Vous pouvez maintenant tout déconnecter et profiter du nouveau logiciel !

## Mise à niveau depuis Linux

Téléchargez *cflash* : `$ curl https://github.com/windbird-sensor/windbird-fox-firmware/raw/main/usb-flasher/linux/cflash.c -o cflash.c`

Compilez-le : `$ gcc cflash.c -o cflash-linux`

Branchez le câble USB entre le Windbird et votre ordinateur.

Identifiez votre port série en exécutant `find /dev -name "ttyUSB*" -o -name "ttyACM*"`.

A cette étape, assurez-vous que votre Windbird est éteint et que la batterie est connectée.

Lancez cflash `$ ./cflash-linux -d /dev/ttyUSB0 myfirmware.bin` (remplacez /dev/ttyUSB0 par le chemin de votre port série).

Allumez le Windbird sous tension en appuyant sur le bouton.

Attendez quelques secondes. Le processus de mise à jour va commencer. Ne touchez à rien, ne quittez rien et ne déconnectez rien pendant la mise à jour.

Lorsque vous lisez *Upgrade OK*, la Windbird est prête et va redémarrer. Vous pouvez maintenant tout déconnecter et profiter du nouveau logiciel !

## Mise à niveau à partir de Mac

Ça devrait être possible.

Mais je n'ai pas réussi à faire fonctionner `cflash` sur Mac.

Voir https://github.com/windbird-sensor/windbird-fox-firmware/blob/main/usb-flasher/mac/compilation.md

Toute aide est la bienvenue
