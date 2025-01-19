import { defineConfig } from 'vite';
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";

// https://vitejs.dev/config/
export default defineConfig({
  server: {
    hmr: false, // Disable hot reload on save
  },
  plugins: [
    topLevelAwait(),
    wasm()
  ],
  base: './',
    build: {
      emptyOutDir: true,
      rollupOptions: {
        treeshake: false
      }
    }
  }
);