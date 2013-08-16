AsyncMBus
=========

Asynchronous Alpine M-Bus protocol driver

Based on https://github.com/Olstyle/MBus this driver uses attachInterrupt and timer interrupt to count pulse times so it is completely free of 'delay'. 

Don't run long tasks in the interrupt callbacks (onTx, onFault), just set states and check them in the polling loop.

The onMessage callback is called from the driver's loop method, not from an ISR.

