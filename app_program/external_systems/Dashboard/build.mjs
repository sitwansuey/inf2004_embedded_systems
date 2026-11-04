import { build, context } from 'esbuild';
import { fileURLToPath } from 'node:url';

const directory = fileURLToPath(new URL('.', import.meta.url));
const options = {
  absWorkingDir: directory,
  entryPoints: ['src/main.jsx'],
  outfile: 'assets/dashboard.js',
  bundle: true,
  format: 'esm',
  jsx: 'automatic',
  minify: true,
  sourcemap: true,
  target: ['es2020'],
  define: { 'process.env.NODE_ENV': '"production"' },
  logLevel: 'info',
};

if (process.argv.includes('--watch')) {
  const builder = await context(options);
  await builder.watch();
  console.log('Watching dashboard sources. Serve with dashboard.py in another terminal.');
} else {
  await build(options);
}
