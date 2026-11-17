import { defineConfig, loadEnv } from 'vite';

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, '.', 'ATD_');
  return {
    server: { proxy: { '/api': { target: env.ATD_API_TARGET || 'http://127.0.0.1:8766', changeOrigin: false } } },
    build: { outDir: 'dist', emptyOutDir: true },
  };
});
