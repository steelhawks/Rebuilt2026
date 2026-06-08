// Thin wrapper around ntcore-ts-client (NT4) for the /tuner overlay.
//
// Loaded straight from a CDN to keep the site build-free, matching how Plotly
// and the rest of src/ are already loaded. Pinned to a known-good version.
import { NetworkTables, NetworkTablesTypeInfos }
  from "https://esm.sh/ntcore-ts-client@3.1.3";

export const T = {
  string: NetworkTablesTypeInfos.kString,
  double: NetworkTablesTypeInfos.kDouble,
  boolean: NetworkTablesTypeInfos.kBoolean,
};

export class NTClient {
  constructor() {
    this.nt = null;
    this.topics = new Map();      // name -> single cached topic (read and/or write)
    this.pubPromises = new Map();  // name -> Promise that resolves once we're the publisher
  }

  /**
   * Connect to an NT4 server. Accepts a bare host ("localhost",
   * "roborio-4414-frc.local"), an IP, a `ws(s)://host:port` URI, or a team
   * number (all digits → resolved via team-number addressing).
   */
  connect(hostOrTeam, onConnection) {
    const raw = String(hostOrTeam || "").trim();
    const cleaned = raw.replace(/^wss?:\/\//i, "").replace(/\/.*$/, "");
    const host = cleaned.replace(/:\d+$/, "") || "localhost";
    const portMatch = cleaned.match(/:(\d+)$/);
    const port = portMatch ? parseInt(portMatch[1], 10) : 5810;

    if (/^\d{1,5}$/.test(host) && !portMatch) {
      this.nt = NetworkTables.getInstanceByTeam(parseInt(host, 10), port);
    } else {
      this.nt = NetworkTables.getInstanceByURI(host, port);
    }
    if (onConnection) this.nt.addRobotConnectionListener(onConnection, true);
    return this.nt;
  }

  isConnected() {
    return !!this.nt && this.nt.isRobotConnected();
  }

  /** Get (or create) the single cached topic object for a name. */
  topic(name, type) {
    let t = this.topics.get(name);
    if (!t) {
      t = this.nt.createTopic(name, type);
      this.topics.set(name, t);
    }
    return t;
  }

  /**
   * Register as publisher of a topic, exactly once. publish() is async in
   * ntcore-ts — the client only becomes the publisher after the server
   * announces the topic — so we cache and await that promise before any write.
   */
  ensurePublisher(name, type) {
    const t = this.topic(name, type);
    if (!this.pubPromises.has(name)) {
      this.pubPromises.set(name, (async () => {
        await t.publish(); // resolves once the server announce arrives
        // The `publisher` flag is flipped by a separate announce listener whose
        // ordering vs. publish()'s resolver isn't guaranteed — wait for it.
        const deadline = Date.now() + 3000;
        while (!t.publisher && Date.now() < deadline) {
          await new Promise((r) => setTimeout(r, 15));
        }
        if (!t.publisher) throw new Error("publisher registration timed out for " + name);
        return t;
      })());
    }
    return this.pubPromises.get(name);
  }

  /** Publish a value. Awaits publisher registration first (no default sent). */
  async set(name, type, value) {
    const t = await this.ensurePublisher(name, type);
    t.setValue(value);
  }

  /** Subscribe to a topic; cb receives the value on every change (and initially). */
  subscribe(name, type, cb) {
    const t = this.topic(name, type);
    t.subscribe((v) => cb(v), { immediate: true });
    return t;
  }
}

/**
 * Java String.hashCode(), replicated exactly so the overlay can confirm the
 * robot parsed byte-for-byte what we pushed. JS strings are UTF-16 like Java's,
 * so iterating charCodeAt with 32-bit overflow matches the JVM.
 */
export function javaHashCode(str) {
  let h = 0;
  for (let i = 0; i < str.length; i++) {
    h = (Math.imul(31, h) + str.charCodeAt(i)) | 0;
  }
  return h;
}
