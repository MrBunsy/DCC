/*
 * Copyright Luke Wallin 2012
 */

package pidcc;

import java.io.IOException;
import java.io.InputStream;

/**
 *
 * @author Luke
 */
public class InputStreamReader implements Runnable {

        private CommLink comm;
        private InputStream in;

        public InputStreamReader(InputStream in,CommLink comm) {
            this.in = in;
            this.comm=comm;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int len = -1;
            try {
                while ((len = this.in.read(buffer)) > -1) {
                    comm.receivedMessage(buffer,len);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
