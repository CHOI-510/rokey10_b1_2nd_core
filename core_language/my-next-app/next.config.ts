import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  async rewrites() {
    return [{
      source: "/api/stt",
      destination: `${process.env.FASTAPI_URL || "http://127.0.0.1:8000"}/api/stt`,
    }];
  },
  /* config options here */
  experimental: {
    agentFeedback: true,
  },
  cacheComponents: true,
  partialPrefetching: true,
  turbopack: {
    rules: {
      "*.css": {
        loaders: ["@tailwindcss/turbopack"],
        as: "*.css",
      },
    },
  },
};

export default nextConfig;
