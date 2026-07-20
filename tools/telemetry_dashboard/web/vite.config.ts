import { defineConfig } from "vite";

export default defineConfig({
  base: "/",
  build: {
    outDir: "../static_app",
    emptyOutDir: true,
    sourcemap: false,
  },
});
