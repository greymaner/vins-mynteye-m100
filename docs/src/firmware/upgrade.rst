.. _firmware_upgrade:

How to upgrade the firmware
============================

Please use the MYNT EYE TOOL to upgrade the firmware.

You can download the firmware and MYNT EYE TOOL installation package in the ``Firmwares`` folder of `MYNTEYE_BOX(Download Link) <http://www.myntai.com/mynteye/s/download>`_ . The file structure is as follows:

.. code-block:: none

  Firmwares/
  ├─Checksum.txt                 # file checksum
  ├─MYNTEYE_S_2.4.0.img          # S1030 firmware
  ├─MYNTEYE_S2100_1.2.img        # S2100 firmware
  ├─...
  └─setup.zip                    # MYNTEYE TOOL zip

The firmware upgrade program currently only supports Windows, so you need to operate under Windows. Proceed as follows:

Download preparation
---------------------

* Download and unzip ``setup.zip``
* Find firmware, such as ``MYNTEYE_S_2.4.0.img``

  * Please refer to :ref:`firmware_applicable` to select the firmware suitable for the SDK version
  * Please refer to ``Checksum.txt`` to find the firmware check code as follows:

    * Run the command in CMD ``certutil -hashfile <*.img> MD5`` .
    * If the check code is incorrect, it means that the download went wrong. Please download it again!

Install MYNT EYE TOOL
---------------------

* Double click on ``setup.msi`` and install the application.

Update Firmware
---------------

* Plug in the MYNT® EYE camera into a USB3.0 port

* Open MYNT EYE TOOL and select ``Options/FirmwareUpdate`` .

.. image:: ../../images/firmware_update_option.png

* Click ``Update`` .

.. image:: ../../images/firmware_update.png
   :width: 60%

* A warning dialog box will pop up, click ``yes`` .

  * This operation will erase the firmware, for details see README.

    * Usually, the MYNT EYE TOOL automatically installs the driver during the upgrade process.
    * If the upgrade fails, refer to README.

.. image:: ../../images/firmware_update_warning.png
   :width: 60%

.. image:: ../../images/firmware_update_dir.png
   :width: 60%

* In the open file selection box, select the firmware you want to upgrade and start upgrading.

.. image:: ../../images/firmware_update_select.png

* Once the upgrade is complete, the status will changes to ``Succeeded``.

.. image:: ../../images/firmware_update_success.png
   :width: 60%

* Close the MYNT EYE TOOL，finish.


.. attention::
  If you can't find MYNT image device,  ``WestBridge_driver``, and ``Cypress USB BootLoader`` at the same time in the device manager, try another computer to perform the above operation. If you can not upgrade successfully, please contact us in time.


Manually update drivers
------------------------

* If the application indicates that you failed to update, you may fail to install the driver automatically. You can try to install the driver manually and then update it. The following is the manual installation of the driver.

* Open device manager, locate ``WestBridge_driver`` device, and right click Update Driver,select ``[application directory]WestBridge_driver\\[corresponding system folders](If it is more than win7, choose wlh)\\[system bits]`` .

.. image:: ../../images/firmware_update_westbridge.png

* For example,if it is the win10 64 bit system computer,and the application is installed under the default path,you should select ``C:\Program Files (x86)\slightech\MYNT EYE TOOL 2.0\WestBridge_driver\wlh\x64``.

* After the installation driver is successful, you can find the ``Cypress USB BootLoader`` device in the device manager.

.. image:: ../../images/firmware_update_cypressUSB.png

* Then plug in the camera and open the application again to update.

.. warning::

  During the first time you open the MYNT® EYE camera after a firmware update, please hold the camera steadily for 3 seconds, for a zero drift compensation process. You can also call the API ``RunOptionAction(Option::ZERO_DRIFT_CALIBRATION)`` for zero drift correction.

.. ::

  .. image:: ../../images/firmware_update_driver.png
  .. image:: ../../images/firmware_update_driver_install.png
